#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <LittleFS.h>
#include "Logger.hpp"

/**
 * StreamingFileCache - Chunked file caching for large files
 *
 * Copies chunks from SD to flash on-demand using ping-pong buffers.
 * Allows files larger than flash by overwriting old chunks as they're consumed.
 * Provides fast startup by only waiting for the first chunk.
 */
class StreamingFileCache {
public:
    static constexpr size_t CHUNK_SIZE = 16384;      // 16KB per chunk
    static constexpr size_t COPY_BUF_SIZE = 4096;    // 4KB copy buffer

    StreamingFileCache() = default;

    ~StreamingFileCache() {
        close();
    }

    /**
     * Open a file for streaming cached reads.
     * Copies first chunk to flash and returns true when ready to read.
     */
    bool open(const char* srcPath, float maxRho) {
        close();

        // Allocate copy buffer
        m_copyBuf = (uint8_t*)malloc(COPY_BUF_SIZE);
        if (!m_copyBuf) {
            LOG("StreamingFileCache: Failed to allocate copy buffer\r\n");
            return false;
        }

        m_srcFile = SD.open(srcPath, FILE_READ);
        if (!m_srcFile) {
            LOG("StreamingFileCache: Failed to open source: %s\r\n", srcPath);
            close(); // Free buffer
            return false;
        }

        m_srcFileSize = m_srcFile.size();
        m_srcBytesRead = 0;
        m_currentChunk = 0;
        m_maxRho = maxRho;
        m_eof = false;

        LOG("StreamingFileCache: Opened %s (%u bytes)\r\n", srcPath, m_srcFileSize);

        // Copy first chunk
        if (!copyNextChunk()) {
            LOG("StreamingFileCache: Failed to copy first chunk\r\n");
            close();
            return false;
        }

        // Open the cached chunk for reading
        if (!openCurrentChunk()) {
            close();
            return false;
        }

        // Pre-copy second chunk if file is large enough
        if (m_srcBytesRead < m_srcFileSize) {
            m_nextChunkReady = copyChunkTo(1 - m_currentChunk);
        }

        return true;
    }

    void close() {
        if (m_srcFile) {
            m_srcFile.close();
        }
        if (m_cacheFile) {
            m_cacheFile.close();
        }
        if (m_copyDest) {
            m_copyDest.close();
        }
        if (m_copyBuf) {
            free(m_copyBuf);
            m_copyBuf = nullptr;
        }
        m_srcFileSize = 0;
        m_srcBytesRead = 0;
        m_copyTargetChunk = -1;
        m_copyChunkBytesWritten = 0;
        m_eof = true;
    }

    /**
     * Read a line from the cached file.
     * Handles chunk transitions transparently.
     * Returns empty string on EOF.
     */
    String readLine() {
        if (m_eof) return "";

        String line;
        line.reserve(64);

        while (true) {
            // Refill internal buffer if needed
            if (m_bufHead >= m_bufTail) {
                if (!fillBuffer()) {
                    // No more data in current chunk - try switching
                    if (!switchToNextChunk()) {
                        m_eof = true;
                        break;
                    }
                    continue;
                }
            }

            // Scan for newline
            bool foundNewline = false;
            int i = m_bufHead;
            for (; i < m_bufTail; i++) {
                if (m_buffer[i] == '\n') {
                    foundNewline = true;
                    break;
                }
            }

            // Append bytes to line
            for (int k = m_bufHead; k < i; k++) {
                line += (char)m_buffer[k];
            }
            m_bufHead = i;

            if (foundNewline) {
                m_bufHead++; // Skip newline
                break;
            }

            // Line length safety limit
            if (line.length() > 256) break;
        }

        return line;
    }

    bool isEOF() const { return m_eof; }

    float getMaxRho() const { return m_maxRho; }

    /**
     * Call periodically to ensure next chunk is pre-loaded.
     * Copies in small increments to avoid blocking for too long.
     * Returns true if there's more work to do (call again soon).
     */
    bool maintainCache() {
        // If we don't have next chunk ready and there's more source data, copy incrementally
        if (!m_nextChunkReady && m_srcBytesRead < m_srcFileSize) {
            // Copy a small increment each call (non-blocking style)
            bool chunkComplete = copyChunkIncremental(1 - m_currentChunk);
            if (chunkComplete) {
                m_nextChunkReady = true;
            }
            return true; // More work to do
        }
        return false;
    }

private:
    File m_srcFile;                    // Source file on SD
    File m_cacheFile;                  // Currently open cache file
    size_t m_srcFileSize = 0;
    size_t m_srcBytesRead = 0;
    int m_currentChunk = 0;            // 0 or 1
    bool m_nextChunkReady = false;
    bool m_eof = true;
    float m_maxRho = 450.0f;
    uint8_t* m_copyBuf = nullptr;      // Heap-allocated copy buffer

    // Incremental copy state
    File m_copyDest;                   // Destination file for incremental copy
    int m_copyTargetChunk = -1;        // Which chunk we're copying to (-1 = none)
    size_t m_copyChunkBytesWritten = 0;// Bytes written to current chunk

    // Read buffer
    static constexpr size_t BUF_SIZE = 1024;
    uint8_t m_buffer[BUF_SIZE];
    int m_bufHead = 0;
    int m_bufTail = 0;

    const char* getChunkPath(int chunk) const {
        return chunk == 0 ? "/cache0.thr" : "/cache1.thr";
    }

    bool copyNextChunk() {
        return copyChunkTo(m_currentChunk);
    }

    bool copyChunkTo(int chunkIdx) {
        if (!m_srcFile || m_srcBytesRead >= m_srcFileSize || !m_copyBuf) {
            return false;
        }

        const char* cachePath = getChunkPath(chunkIdx);

        // Remove old file if exists
        if (LittleFS.exists(cachePath)) {
            LittleFS.remove(cachePath);
        }

        File dest = LittleFS.open(cachePath, FILE_WRITE);
        if (!dest) {
            LOG("StreamingFileCache: Failed to create cache file %s\r\n", cachePath);
            return false;
        }

        size_t remaining = CHUNK_SIZE;
        size_t chunkBytesWritten = 0;

        while (remaining > 0 && m_srcFile.available()) {
            size_t toRead = (remaining < COPY_BUF_SIZE) ? remaining : COPY_BUF_SIZE;
            size_t n = m_srcFile.read(m_copyBuf, toRead);
            if (n == 0) break;

            dest.write(m_copyBuf, n);
            remaining -= n;
            m_srcBytesRead += n;
            chunkBytesWritten += n;
        }

        dest.close();

        LOG("StreamingFileCache: Copied chunk %d (%u bytes, %u/%u total)\r\n",
            chunkIdx, chunkBytesWritten, m_srcBytesRead, m_srcFileSize);

        return chunkBytesWritten > 0;
    }

    /**
     * Copy a small increment of the next chunk.
     * Call repeatedly until it returns true (chunk complete).
     * Only blocks for ~4KB of SD read at a time.
     */
    bool copyChunkIncremental(int chunkIdx) {
        if (!m_copyBuf) return false;

        // Start new chunk copy if needed
        if (m_copyTargetChunk != chunkIdx) {
            // Close any previous copy in progress
            if (m_copyDest) {
                m_copyDest.close();
            }

            const char* cachePath = getChunkPath(chunkIdx);

            // Remove old file if exists
            if (LittleFS.exists(cachePath)) {
                LittleFS.remove(cachePath);
            }

            m_copyDest = LittleFS.open(cachePath, FILE_WRITE);
            if (!m_copyDest) {
                LOG("StreamingFileCache: Failed to create cache file %s\r\n", cachePath);
                return true; // Return true to stop retrying
            }

            m_copyTargetChunk = chunkIdx;
            m_copyChunkBytesWritten = 0;
        }

        // Copy one increment (COPY_BUF_SIZE bytes)
        if (m_srcFile.available() && m_copyChunkBytesWritten < CHUNK_SIZE) {
            size_t remaining = CHUNK_SIZE - m_copyChunkBytesWritten;
            size_t toRead = (remaining < COPY_BUF_SIZE) ? remaining : COPY_BUF_SIZE;

            size_t n = m_srcFile.read(m_copyBuf, toRead);
            if (n > 0) {
                m_copyDest.write(m_copyBuf, n);
                m_copyChunkBytesWritten += n;
                m_srcBytesRead += n;
            }
        }

        // Check if chunk is complete
        if (m_copyChunkBytesWritten >= CHUNK_SIZE || !m_srcFile.available()) {
            m_copyDest.close();
            LOG("StreamingFileCache: Incremental copy chunk %d complete (%u bytes)\r\n",
                chunkIdx, m_copyChunkBytesWritten);
            m_copyTargetChunk = -1;
            return true; // Chunk complete
        }

        return false; // More to copy
    }

    bool openCurrentChunk() {
        const char* cachePath = getChunkPath(m_currentChunk);
        m_cacheFile = LittleFS.open(cachePath, FILE_READ);
        if (!m_cacheFile) {
            LOG("StreamingFileCache: Failed to open cache %s for reading\r\n", cachePath);
            return false;
        }
        m_bufHead = 0;
        m_bufTail = 0;
        return true;
    }

    bool switchToNextChunk() {
        // Close current chunk
        if (m_cacheFile) {
            m_cacheFile.close();
        }

        // If next chunk isn't ready and there's more data, copy it now
        if (!m_nextChunkReady) {
            if (m_srcBytesRead >= m_srcFileSize) {
                // No more source data - true EOF
                return false;
            }
            // Emergency sync copy (shouldn't happen if maintainCache is called regularly)
            LOG("StreamingFileCache: Emergency chunk copy (cache underrun)\r\n");
            m_nextChunkReady = copyChunkTo(1 - m_currentChunk);
            if (!m_nextChunkReady) {
                return false;
            }
        }

        // Switch to next chunk
        m_currentChunk = 1 - m_currentChunk;
        m_nextChunkReady = false;

        if (!openCurrentChunk()) {
            return false;
        }

        // Start copying the next chunk in background (will be done by maintainCache)
        return true;
    }

    bool fillBuffer() {
        if (!m_cacheFile) return false;

        // Move remaining data to front
        if (m_bufHead > 0 && m_bufHead < m_bufTail) {
            memmove(m_buffer, m_buffer + m_bufHead, m_bufTail - m_bufHead);
            m_bufTail -= m_bufHead;
            m_bufHead = 0;
        } else if (m_bufHead >= m_bufTail) {
            m_bufHead = 0;
            m_bufTail = 0;
        }

        // Read more from cache file
        if (m_bufTail < BUF_SIZE && m_cacheFile.available()) {
            int toRead = BUF_SIZE - m_bufTail;
            int bytesRead = m_cacheFile.read(m_buffer + m_bufTail, toRead);
            if (bytesRead > 0) {
                m_bufTail += bytesRead;
                return true;
            }
        }

        return m_bufTail > m_bufHead; // Return true if there's still data in buffer
    }
};
