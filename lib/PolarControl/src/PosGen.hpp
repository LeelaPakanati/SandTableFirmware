#pragma once
#include <Arduino.h>
#include <SDCard.hpp>
#include <FS.h>
#include <cmath>
#include "PolarUtils.hpp"
#include "Logger.hpp"

class PosGen {
  public:
    virtual ~PosGen() = default;
    virtual PolarCord_t getNextPos() = 0;

    // Progress tracking: returns 0-100 percentage, -1 if unknown
    virtual int getProgressPercent() const { return -1; }
};

class FilePosGen : public PosGen {
  public:
    ~FilePosGen() {
      if (m_file) {
        m_file.close();
      }
    }

    FilePosGen(fs::FS &fs, String filePath, float maxRho = 450.0) :
      m_fs(fs),
      m_currFile(filePath),
      m_maxRho(maxRho),
      m_bufHead(0),
      m_bufTail(0)
    {
      m_file = m_fs.open(filePath.c_str(), FILE_READ);
      if (!m_file) {
        LOG("ERROR: Could not open file: %s\r\n", filePath.c_str());
      } else {
        m_fileSize = m_file.size();
        LOG("Opened file: %s (Size: %u bytes)\r\n", filePath.c_str(), m_fileSize);
        fillBuffer(); // Initial fill
      }
    }

    PolarCord_t getNextPos() override {
      if (!m_file && m_bufHead == m_bufTail) {
        return {std::nanf(""), std::nanf("")};
      }

      // Read a line from the buffer
      String line = readBufferedLine();
      if (line.length() == 0 && !m_file && m_bufHead == m_bufTail) {
          // EOF
          return {std::nanf(""), std::nanf("")};
      }
      
      line.trim();
      m_currLine++;

      // Skip empty lines and comments
      if (line.length() == 0 || line.startsWith("#") || line.startsWith("//")) {
        return getNextPos();
      }

      // Parse the line: expected format "theta rho" or "theta,rho"
      float theta = 0.0;
      float rho = 0.0;

      int separatorIndex = line.indexOf(',');
      if (separatorIndex == -1) {
        separatorIndex = line.indexOf(' ');
      }
      if (separatorIndex == -1) {
        separatorIndex = line.indexOf('\t');
      }

      if (separatorIndex != -1) {
        String thetaStr = line.substring(0, separatorIndex);
        String rhoStr = line.substring(separatorIndex + 1);
        thetaStr.trim();
        rhoStr.trim();

        theta = thetaStr.toFloat();
        rho = rhoStr.toFloat();

        // Scale rho from 0-1 range to 0-maxRho
        rho = rho * m_maxRho;

        return {theta, rho};
      } else {
        LOG("ERROR: Invalid line format at line %d: %s\r\n", m_currLine, line.c_str());
        return getNextPos();
      }
    }

    int getProgressPercent() const override {
      if (!m_file || m_fileSize == 0) return -1;
      return (int)((m_file.position() * 100) / m_fileSize);
    }

  private:
    fs::FS &m_fs;
    int m_currLine = 0;
    String m_currFile = "";
    float m_maxRho;
    File m_file;
    size_t m_fileSize = 0;

    static constexpr size_t BUF_SIZE = 1024;
    uint8_t m_buffer[BUF_SIZE];
    int m_bufHead = 0;
    int m_bufTail = 0;

    void fillBuffer() {
        if (!m_file || !m_file.available()) return;
        
        // Move existing data to front if needed
        if (m_bufHead > 0 && m_bufHead < m_bufTail) {
            memmove(m_buffer, m_buffer + m_bufHead, m_bufTail - m_bufHead);
            m_bufTail -= m_bufHead;
            m_bufHead = 0;
        } else if (m_bufHead == m_bufTail) {
            m_bufHead = 0;
            m_bufTail = 0;
        }

        // Read more
        if (m_bufTail < BUF_SIZE) {
            int toRead = BUF_SIZE - m_bufTail;
            int bytesRead = m_file.read(m_buffer + m_bufTail, toRead);
            if (bytesRead > 0) {
                m_bufTail += bytesRead;
            } else if (bytesRead < 0) {
                // Error or end
                m_file.close();
            }
        }
    }

    String readBufferedLine() {
        String line = "";
        line.reserve(64);
        
        while (true) {
            // Refill if empty
            if (m_bufHead >= m_bufTail) {
                fillBuffer();
                if (m_bufHead >= m_bufTail) break; // EOF
            }

            // Scan for newline
            bool foundNewline = false;
            int i = m_bufHead;
            for (; i < m_bufTail; i++) {
                char c = (char)m_buffer[i];
                if (c == '\n') {
                    foundNewline = true;
                    break;
                }
            }

            // Append bytes to line safe and clean
            for (int k = m_bufHead; k < i; k++) {
                line += (char)m_buffer[k];
            }

            m_bufHead = i;

            if (foundNewline) {
                m_bufHead++; // Skip the newline
                break;
            }

            // No newline in current buffer, we consumed it all
            // Loop will continue and refill buffer
            
            // Check max line length to prevent memory issues with weird files
            if (line.length() > 256) break; 
        }
        return line;
    }
};
