//
// Created by manuelli on 1/20/17.
//

#ifndef DRAKE_DISTRO_LCMHANDLER_H
#define DRAKE_DISTRO_LCMHANDLER_H

#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <iostream>
#include <stdexcept>


namespace drake {
  namespace examples {
    namespace ContactParticleFilter {

      class LCMHandler {

      public:

        std::thread ThreadHandle;
        std::shared_ptr <lcm::LCM> LCMHandle;
        bool ShouldStop;

        LCMHandler() {
          this->ShouldStop = false;
          this->InitLCM();
        }

        void InitLCM() {
          this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
          if (!this->LCMHandle->good()) {
            std::cout << "ERROR: lcm is not good()" << std::endl;
          }
        }

        bool WaitForLCM(double timeout) {
          int lcmFd = this->LCMHandle->getFileno();

          timeval tv;
          tv.tv_sec = 0;
          tv.tv_usec = timeout * 1e6;

          fd_set fds;
          FD_ZERO(&fds);
          FD_SET(lcmFd, &fds);

          int status = select(lcmFd + 1, &fds, 0, 0, &tv);
          if (status == -1 && errno != EINTR) {
            printf("select() returned error: %d\n", errno);
          } else if (status == -1 && errno == EINTR) {
            printf("select() interrupted\n");
          }
          return (status > 0 && FD_ISSET(lcmFd, &fds));
        }

        void ThreadLoopWithSelect() {

          std::cout << "ThreadLoopWithSelect " << std::this_thread::get_id() << std::endl;

          while (!this->ShouldStop) {
            const double timeoutInSeconds = 0.3;
            bool lcmReady = this->WaitForLCM(timeoutInSeconds);

            if (this->ShouldStop) {
              break;
            }

            if (lcmReady) {
              if (this->LCMHandle->handle() != 0) {
                printf("lcm->handle() returned non-zero\n");
                break;
              }
            }
          }

          std::cout << "ThreadLoopWithSelect ended " << std::this_thread::get_id() << std::endl;

        }

        void ThreadLoop() {
          while (!this->ShouldStop) {
            if (this->LCMHandle->handle() != 0) {
              printf("lcm->handle() returned non-zero\n");
              break;
            }
          }
        }

        bool IsRunning() {
          return this->ThreadHandle.joinable();
        }

        void Start() {
          std::cout << "LCMHandler start... " << std::this_thread::get_id() << std::endl;
          if (this->IsRunning()) {
            std::cout << "already running lcm thread. " << std::this_thread::get_id() << std::endl;
            return;
          }

          this->ShouldStop = false;
          this->ThreadHandle = std::thread(&LCMHandler::ThreadLoopWithSelect, this);
        }

        void Stop() {
          this->ShouldStop = true;
          this->ThreadHandle.join();
        }

      };
}// ContactParticleFilter
}// examples
}// drake

#endif //DRAKE_DISTRO_LCMHANDLER_H