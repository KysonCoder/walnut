#include <signal.h>
#include <unistd.h>
#include <iostream>
#include "launcher.hpp"

static Launcher* g_launcher = nullptr;

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        std::cout << "Received signal " << sig << ", shutting down...\n";
        if (g_launcher) {
            g_launcher->stop();
        }
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    Launcher launcher;
    g_launcher = &launcher;

    std::string config_path;
    bool use_default = false;
    if (argc > 1) {
        config_path = argv[1];
    } else if (access("cfg/taskframe_launcher.yaml", F_OK) == 0) {
        config_path = "cfg/taskframe_launcher.yaml";
    } else if (access("taskframe_launcher.yaml", F_OK) == 0) {
        // Backward compatibility: allow old location
        config_path = "taskframe_launcher.yaml";
    } else {
        use_default = true;
    }

    if (use_default) {
        std::cout << "No config file found, using default built-in config\n";
        if (!launcher.init_with_default()) {
            std::cerr << "Launcher init failed\n";
            return 1;
        }
    } else {
        if (!launcher.init(config_path)) {
            std::cerr << "Launcher init failed\n";
            return 1;
        }
    }

    if (!launcher.start_all_workers()) {
        std::cerr << "Failed to start workers\n";
        return 1;
    }

    while (launcher.is_running()) {
        launcher.run_once();
        usleep(100000);  // 10ms
        if (getchar() != EOF) {

        std::cout << "The system is shutting down. Please wait for a moment."
                  << std::endl;
        break;
        }
    }

    launcher.stop_all_workers();
    g_launcher = nullptr;
    return 0;
}