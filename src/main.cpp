#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include "kinetic-delaunay/controller/manager.h"
#include <memory>
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    //TODO(yue) common configurations
    int X_OFFSET = 200;
    int Y_OFFSET = 400;
    double BOUND = 300;
    double FACTOR = 10000;
    int FPS = 50;
    double DELTA = 1.0 / FPS;
    int MAX_FRMAE_SKIPS = 5;
    long PERIOD = 1000000L / FPS;
    String DATASET_PATH = "../../dataset/sample3.txt";

    //TODO(yue) visualization window
    auto manager = Manager(BOUND, DELTA, FACTOR);
    manager.Load(DATASET_PATH);
    String windowName = "visualization";
    namedWindow(windowName);
    moveWindow(windowName, X_OFFSET, Y_OFFSET);

    auto beforeTime = std::chrono::high_resolution_clock::now();
    long excess = 0L;
    auto overSleepTime = 0L;

    //TODO(yue) animation procedures
    while (true) {
        manager.Update();
        imshow(windowName, manager.Render());

        int k = waitKey((int) (DELTA * 1000 - 1));
        if (k >= 0) break;

        auto afterTime = std::chrono::high_resolution_clock::now();
        long timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(afterTime - beforeTime).count();
        long sleepTime = (PERIOD - timeDiff) - overSleepTime;

        if (sleepTime > 0) { //TODO(yue) release the time left in this cycle
            std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
        } else { //TODO(yue) store excess time from this cycle
            excess -= sleepTime;
            overSleepTime = 0L;
        }

        beforeTime = std::chrono::high_resolution_clock::now();
        int skips = 0;
        while ((excess > PERIOD) && (skips < MAX_FRMAE_SKIPS)) {
            excess -= PERIOD;
            manager.Update();
            skips += 1;
        }
    }
    destroyWindow(windowName);
    return 0;
}
