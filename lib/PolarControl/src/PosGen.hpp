#pragma once
#include <Arduino.h>
#include <SDCard.hpp>
#include <cmath>
#include "PolarUtils.hpp"

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

    FilePosGen(String filePath, double maxRho = 450.0) :
      m_currFile(filePath),
      m_maxRho(maxRho)
    {
      m_file = SD.open(filePath.c_str(), FILE_READ);
      if (!m_file) {
        Serial.print("ERROR: Could not open file: ");
        Serial.println(filePath);
      } else {
        m_fileSize = m_file.size();
        Serial.print("Opened file: ");
        Serial.print(filePath);
        Serial.print(" (Size: ");
        Serial.print(m_fileSize);
        Serial.println(" bytes)");
      }
    }

    PolarCord_t getNextPos() override {
      if (!m_file || !m_file.available()) {
        if (m_file) {
          m_file.close();
          Serial.println("End of file reached");
        }
        return {std::nan(""), std::nan("")};
      }

      // Read a line from the file
      String line = m_file.readStringUntil('\n');
      line.trim();
      m_currLine++;

      // Skip empty lines and comments
      if (line.length() == 0 || line.startsWith("#") || line.startsWith("//")) {
        return getNextPos();
      }

      // Parse the line: expected format "theta rho" or "theta,rho"
      double theta = 0.0;
      double rho = 0.0;

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

        theta = thetaStr.toDouble();
        rho = rhoStr.toDouble();

        // Scale rho from 0-1 range to 0-maxRho
        rho = rho * m_maxRho;

        return {theta, rho};
      } else {
        Serial.print("ERROR: Invalid line format at line ");
        Serial.print(m_currLine);
        Serial.print(": ");
        Serial.println(line);
        return getNextPos();
      }
    }

    int getProgressPercent() const override {
      if (!m_file || m_fileSize == 0) return -1;
      return (int)((m_file.position() * 100) / m_fileSize);
    }

  private:
    int m_currLine = 0;
    String m_currFile = "";
    double m_maxRho;
    File m_file;
    size_t m_fileSize = 0;
};
