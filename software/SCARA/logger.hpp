#ifndef LOGGER_HPP
#define LOGGER_HPP


class Logger {
public:
    enum LogLevel {
        INFO,
        WARN,
        ERROR,
        DEBUG
    };

    static void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

    template<typename... Args>
    static void log(LogLevel level, const char* format, Args... args) {
        /**
         * Logging function with log level and variable arguments.
         */

        if (level <= currentLogLevel) {
            String formattedMessage = formatString(format, args...);
            Serial.print(currentTime());
            Serial.print(" [");
            Serial.print(logLevelToString(level));
            Serial.print("]: ");
            Serial.println(formattedMessage);
        }
    }

    // Logging functions for specific log levels
    template<typename... Args>
    static void info(const char* format, Args... args) {
        log(INFO, format, args...);
    }

    template<typename... Args>
    static void debug(const char* format, Args... args) {
        log(DEBUG, format, args...);
    }

    template<typename... Args>
    static void warn(const char* format, Args... args) {
        log(WARN, format, args...);
    }

    template<typename... Args>
    static void error(const char* format, Args... args) {
        log(ERROR, format, args...);
    }

private:

    static LogLevel currentLogLevel;

    template<typename T, typename... Args>
    static void formatToString(String &result, const char* format, T value, Args... args) {
        /**
         * Recursively format the string with arguments.
         */

        const char* placeholder = strstr(format, "{}");
        if (placeholder != nullptr) {
            result += String(format).substring(0, placeholder - format); // Before {}
            result += String(value); // Insert argument
            formatToString(result, placeholder + 2, args...); // Process next argument
        } else {
            result += String(format); // Append remaining string if no more placeholders
        }
    }

    static void formatToString(String &result, const char* format) {
        /**
         * Base case: when no more arguments are left, just append the rest of the format.
         */

        result += String(format);
    }

    template<typename... Args>
    static String formatString(const char* format, Args... args) {
        /**
         * Helper to kick off the recursive formatting.
         */

        String result = "";
        formatToString(result, format, args...);
        return result;
    }

    static const char* logLevelToString(LogLevel level) {
        /**
         * Convert log level to string.
         */

        switch (level) {
            case INFO:  return "INFO";
            case WARN:  return "WARN";
            case DEBUG: return "DEBUG";
            case ERROR: return "ERROR";
            default:    return "UNKNOWN";
        }
    }

    static String currentTime() {
        /**
         * Get current time for the log entry (formatted as mm:ss, padded to 10 chars).
         */
        unsigned long now = millis() / 1000;
        unsigned int minutes = now / 60;
        unsigned int seconds = now % 60;

        // Format minutes and seconds as "mm:ss"
        char buffer[11]; // 10 chars + null terminator
        snprintf(buffer, sizeof(buffer), "%4d:%02d ", minutes, seconds);

        return String(buffer);
    }
};

// Initialize the static member
Logger::LogLevel Logger::currentLogLevel = Logger::INFO;

#endif // LOGGER_HPP
