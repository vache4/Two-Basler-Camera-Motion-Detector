// badminton_landing_detector.cpp
// C++17 port of badminton_landing_detector_2.0(1).py
// Target: Windows + MSVC + Basler pylon + OpenCV

#include <pylon/PylonIncludes.h>
#include <GenApi/GenApi.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace Pylon;
using namespace GenApi;
namespace fs = std::filesystem;

static const std::string CAMERA_A_SERIAL = "REPLACE_WITH_CAMERA_A_SERIAL";
static const std::string CAMERA_B_SERIAL = "REPLACE_WITH_CAMERA_B_SERIAL";
static constexpr double MM_PER_PX = 550.0 / 400.0;
static constexpr double GAIN = 20.0;
static constexpr double FPS = 100.0;
static constexpr double EXPOSURE = 100.0;
static constexpr int CAM_A_HEIGHT = 100;
static constexpr int CAM_B_HEIGHT = 400;
static constexpr double DISPLAY_FPS = 5.0;
static constexpr double DISPLAY_INT = 1.0 / DISPLAY_FPS;

static constexpr int MOTION_THRESHOLD = 25;
static constexpr double MOTION_MIN_AREA_FRAC = 0.02;
static constexpr double REARM_DELAY_S = 1.0;

static constexpr int WHITE_BRIGHTNESS_THRESH = 150;
static constexpr double CIRCULARITY_MIN = 0.50;
static constexpr double SKIRT_R_PX = 65.0 / 2.0 / MM_PER_PX;
static constexpr double MIN_RADIUS_PX = 0.8 * SKIRT_R_PX;
static constexpr double MAX_RADIUS_PX = 1.4 * SKIRT_R_PX;

static constexpr int BLACK_BRIGHTNESS_THRESH = 100;
static constexpr double BLACK_CIRCULARITY_MIN = 0.40;
static constexpr double BLACK_R_PX = 15.0 / 2.0 / MM_PER_PX;
static constexpr double BLACK_MIN_RADIUS_PX = 0.6 * BLACK_R_PX;
static constexpr double BLACK_MAX_RADIUS_PX = 1.7 * BLACK_R_PX;
static constexpr double BLACK_SEARCH_RADIUS_FRAC = 0.90;

static constexpr double WHITE_CANDIDATE_ALPHA = 0.40;
static constexpr double BLACK_CANDIDATE_ALPHA = 0.70;

static constexpr int BEEP_FREQUENCY_HZ = 1000;
static constexpr int BEEP_DURATION_MS = 150;

static std::tm localTime(std::time_t t) {
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    return tm;
}

static std::string timestamp(const char* fmt, bool millis=false) {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = localTime(t);
    std::ostringstream ss;
    ss << std::put_time(&tm, fmt);
    if (millis) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    }
    return ss.str();
}

static const std::string RUN_TS = timestamp("%Y-%m-%d %H.%M.%S");
static const fs::path SAVE_DIR = fs::path("captures2") / RUN_TS;
static const fs::path COORD_CSV_PATH = fs::path("captures2") / (RUN_TS + "_coords.csv");

static void setEnum(INodeMap& nm, const char* name, const char* value, bool required=false) {
    CEnumerationPtr n(nm.GetNode(name));
    if (!IsAvailable(n) || !IsWritable(n)) {
        if (required) throw std::runtime_error(std::string("Node not writable: ") + name);
        return;
    }
    n->FromString(value);
}
static void setBool(INodeMap& nm, const char* name, bool value) {
    CBooleanPtr n(nm.GetNode(name));
    if (IsAvailable(n) && IsWritable(n)) n->SetValue(value);
}
static void setFloat(INodeMap& nm, const char* name, double value, bool required=false) {
    CFloatPtr n(nm.GetNode(name));
    if (!IsAvailable(n) || !IsWritable(n)) {
        if (required) throw std::runtime_error(std::string("Node not writable: ") + name);
        return;
    }
    value = std::max(n->GetMin(), std::min(value, n->GetMax()));
    n->SetValue(value);
}
static int64_t alignInt(int64_t v, int64_t minv, int64_t maxv, int64_t inc) {
    v = std::max(minv, std::min(v, maxv));
    return inc > 1 ? minv + ((v - minv) / inc) * inc : v;
}
static void setInt(INodeMap& nm, const char* name, int64_t value, bool required=false) {
    CIntegerPtr n(nm.GetNode(name));
    if (!IsAvailable(n) || !IsWritable(n)) {
        if (required) throw std::runtime_error(std::string("Node not writable: ") + name);
        return;
    }
    n->SetValue(alignInt(value, n->GetMin(), n->GetMax(), n->GetInc()));
}

class CameraStream {
public:
    using Callback = std::function<void(const cv::Mat&)>;

    CameraStream(const CDeviceInfo& info, double fps, double exposure,
                 int height, std::string name, Callback cb=nullptr)
        : name_(std::move(name)), cb_(std::move(cb)),
          camera_(CTlFactory::GetInstance().CreateDevice(info))
    {
        camera_.Open();
        INodeMap& nm = camera_.GetNodeMap();

        setEnum(nm, "ExposureAuto", "Off");
        setFloat(nm, "ExposureTime", exposure, true);
        setEnum(nm, "GainAuto", "Off");
        setFloat(nm, "Gain", GAIN, true);
        setEnum(nm, "PixelFormat", "Mono8", true);
        setInt(nm, "BinningVertical", 1);
        setInt(nm, "BinningHorizontal", 1);
        setBool(nm, "AcquisitionFrameRateEnable", true);
        setFloat(nm, "AcquisitionFrameRate", fps, true);

        CIntegerPtr hnode(nm.GetNode("Height"));
        if (!IsAvailable(hnode) || !IsReadable(hnode))
            throw std::runtime_error("Height node not readable");

        int64_t sensorH = hnode->GetMax();
        setInt(nm, "OffsetY", 0, true);
        setInt(nm, "Height", height, true);
        int64_t actualH = hnode->GetValue();
        setInt(nm, "OffsetY", std::max<int64_t>(0, (sensorH - actualH)/2), true);

        converter_.OutputPixelFormat = PixelType_Mono8;
        converter_.OutputBitAlignment = OutputBitAlignment_MsbAligned;
    }

    ~CameraStream() { stop(); }
    void setOnFrame(Callback cb) { cb_ = std::move(cb); }

    void start() {
        if (running_) return;
        camera_.StartGrabbing(GrabStrategy_LatestImageOnly);
        running_ = true;
        thread_ = std::thread(&CameraStream::grabLoop, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (camera_.IsGrabbing()) camera_.StopGrabbing();
        if (camera_.IsOpen()) camera_.Close();
    }

    cv::Mat getLatest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_.empty() ? cv::Mat() : latest_.clone();
    }
    cv::Mat getDisplay() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return display_.empty() ? cv::Mat() : display_.clone();
    }

private:
    void grabLoop() {
        auto lastDisplay = std::chrono::steady_clock::time_point{};
        while (running_ && camera_.IsGrabbing()) {
            CGrabResultPtr grab;
            try {
                camera_.RetrieveResult(1000, grab, TimeoutHandling_ThrowException);
            } catch (const TimeoutException&) {
                continue;
            }

            if (!grab || !grab->GrabSucceeded()) continue;

            CPylonImage converted;
            converter_.Convert(converted, grab);

            cv::Mat view((int)converted.GetHeight(), (int)converted.GetWidth(),
                         CV_8UC1, converted.GetBuffer());
            cv::Mat frame = view.clone();
            cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);

            auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = frame;
                if (std::chrono::duration<double>(now - lastDisplay).count() >= DISPLAY_INT) {
                    display_ = frame;
                    lastDisplay = now;
                }
            }
            if (cb_) cb_(frame);
        }
    }

    std::string name_;
    Callback cb_;
    CInstantCamera camera_;
    CImageFormatConverter converter_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    cv::Mat latest_, display_;
};

struct ShuttlecockDetection {
    double whiteX, whiteY, whiteRadius;
    std::optional<double> blackX, blackY, blackRadius, centerDistance;
};

struct DetectionResult {
    std::optional<ShuttlecockDetection> shcock;
    cv::Mat whiteMask, blackMask;
};

static std::pair<std::optional<cv::Vec3f>, cv::Mat>
detectWhiteCircle(const cv::Mat& frame) {
    cv::Mat gray = frame.channels()==1 ? frame : cv::Mat();
    if (frame.channels()!=1) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurred, thresh;
    cv::GaussianBlur(gray, blurred, cv::Size(3,3), 2);
    cv::threshold(blurred, thresh, WHITE_BRIGHTNESS_THRESH, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::optional<cv::Vec3f> best;
    double bestCircularity = 0.0;

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        double perimeter = cv::arcLength(c, true);
        if (area < 10 || perimeter == 0) continue;

        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < CIRCULARITY_MIN) continue;

        cv::Point2f center; float radius;
        cv::minEnclosingCircle(c, center, radius);
        if (radius < MIN_RADIUS_PX || radius > MAX_RADIUS_PX) continue;

        if (circularity > bestCircularity) {
            bestCircularity = circularity;
            best = cv::Vec3f(center.x, center.y, radius);
        }
    }
    return {best, thresh};
}

static std::pair<std::optional<cv::Vec3f>, cv::Mat>
detectBlackCircle(const cv::Mat& frame, const cv::Vec3f& white) {
    double xw=white[0], yw=white[1], rw=white[2];

    cv::Mat gray = frame.channels()==1 ? frame : cv::Mat();
    if (frame.channels()!=1) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3,3), 1);

    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    cv::circle(mask, cv::Point((int)xw,(int)yw),
               (int)(rw*BLACK_SEARCH_RADIUS_FRAC), cv::Scalar(255), -1);

    cv::Mat dark;
    cv::threshold(blurred, dark, BLACK_BRIGHTNESS_THRESH, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(dark, mask, dark);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::optional<cv::Vec3f> best;
    double bestScore = -1.0;

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        double perimeter = cv::arcLength(c, true);
        if (area < 10 || perimeter == 0) continue;

        double circularity = 4.0 * CV_PI * area / (perimeter*perimeter);
        if (circularity < BLACK_CIRCULARITY_MIN) continue;

        cv::Point2f center; float radius;
        cv::minEnclosingCircle(c, center, radius);
        if (radius < BLACK_MIN_RADIUS_PX || radius > BLACK_MAX_RADIUS_PX) continue;

        double cd = std::hypot(center.x-xw, center.y-yw);
        if (cd + radius > rw) continue;

        double score = circularity * area;
        if (score > bestScore) {
            bestScore = score;
            best = cv::Vec3f(center.x, center.y, radius);
        }
    }
    return {best, dark};
}

static DetectionResult detectShuttlecock(const cv::Mat& frame) {
    DetectionResult r;
    auto [whiteOpt, whiteMask] = detectWhiteCircle(frame);
    r.whiteMask = whiteMask;
    if (!whiteOpt) return r;

    auto white = *whiteOpt;
    ShuttlecockDetection s{
        white[0], white[1], white[2],
        std::nullopt, std::nullopt, std::nullopt, std::nullopt
    };

    auto [blackOpt, blackMask] = detectBlackCircle(frame, white);
    r.blackMask = blackMask;

    if (blackOpt) {
        auto b = *blackOpt;
        s.blackX = b[0]; s.blackY = b[1]; s.blackRadius = b[2];
        s.centerDistance = std::hypot(*s.blackX-s.whiteX, *s.blackY-s.whiteY);
    }

    r.shcock = s;
    return r;
}

static void overlayMask(cv::Mat& display, const cv::Mat& mask,
                        const cv::Scalar& color, double alpha) {
    if (mask.empty()) return;
    for (int y=0; y<mask.rows; ++y) {
        const uchar* m = mask.ptr<uchar>(y);
        cv::Vec3b* d = display.ptr<cv::Vec3b>(y);
        for (int x=0; x<mask.cols; ++x) {
            if (!m[x]) continue;
            for (int c=0; c<3; ++c)
                d[x][c] = cv::saturate_cast<uchar>(
                    d[x][c]*(1.0-alpha) + color[c]*alpha);
        }
    }
}

class CoordinateLogger {
public:
    explicit CoordinateLogger(const fs::path& path) {
        fs::create_directories(path.parent_path());
        bool isNew = !fs::exists(path) || fs::file_size(path)==0;
        file_.open(path, std::ios::app);
        if (!file_) throw std::runtime_error("Could not open CSV");
        if (isNew) {
            file_ << "event,timestamp,white_x_mm,white_y_mm,white_radius_mm,"
                     "black_x_mm,black_y_mm,black_radius_mm,center_distance_mm\n";
            file_.flush();
        }
    }

    void log(int eventId, const std::optional<ShuttlecockDetection>& s) {
        std::lock_guard<std::mutex> lock(mutex_);
        file_ << eventId << "," << timestamp("%Y-%m-%d %H.%M.%S", true) << ",";
        if (!s) {
            file_ << ",,,,,,\n";
            file_.flush();
            return;
        }

        writeMm(s->whiteX); file_ << ",";
        writeMm(s->whiteY); file_ << ",";
        writeMm(s->whiteRadius); file_ << ",";
        if (s->blackX) writeMm(*s->blackX); file_ << ",";
        if (s->blackY) writeMm(*s->blackY); file_ << ",";
        if (s->blackRadius) writeMm(*s->blackRadius); file_ << ",";
        if (s->centerDistance) writeMm(*s->centerDistance);
        file_ << "\n";
        file_.flush();
    }

private:
    void writeMm(double px) {
        file_ << std::fixed << std::setprecision(2) << px * MM_PER_PX;
    }
    std::ofstream file_;
    std::mutex mutex_;
};

static void beepAsync() {
    std::thread([]{
#ifdef _WIN32
        ::Beep(BEEP_FREQUENCY_HZ, BEEP_DURATION_MS);
#else
        std::cout << '\a' << std::flush;
#endif
    }).detach();
}

static void savePair(const cv::Mat& a, const cv::Mat& b, int id) {
    fs::create_directories(SAVE_DIR);

    auto makeName = [id](char prefix) {
        std::ostringstream ss;
        ss << prefix << std::setfill('0') << std::setw(3) << id << ".png";
        return ss.str();
    };

    if (!a.empty()) cv::imwrite((SAVE_DIR / makeName('a')).string(), a);
    if (!b.empty()) cv::imwrite((SAVE_DIR / makeName('b')).string(), b);

    std::cout << "[motion] event #" << id << " - saved still pair ("
              << timestamp("%Y-%m-%d %H.%M.%S", true) << ")\n";
    beepAsync();
}

class MotionDetector {
public:
    using Callback = std::function<void(const cv::Mat&)>;

    MotionDetector(int threshold, double minFrac, double rearmDelay, Callback cb)
        : threshold_(threshold), minFrac_(minFrac),
          rearmDelay_(rearmDelay), cb_(std::move(cb)) {}

    ~MotionDetector() { waitForCallbacks(); }

    void process(const cv::Mat& frame) {
        cv::Mat gray = frame.channels()==1 ? frame.clone() : cv::Mat(); // .clone() added by Valery
        if (frame.channels()!=1) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(21,21), 0);

        bool trigger = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto now = std::chrono::steady_clock::now();
            if (prev_.empty() || now < rearmAt_) {
                prev_ = gray.clone();
                return;
            }

            cv::Mat diff, th;
            cv::absdiff(prev_, gray, diff);
            prev_ = gray.clone();
            cv::threshold(diff, th, threshold_, 255, cv::THRESH_BINARY);

            double changed = (double)cv::countNonZero(th) / (double)th.total();
            if (changed >= minFrac_) {
                rearmAt_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(rearmDelay_));
                trigger = true;
            }
        }

        if (!trigger) return;

        cv::Mat triggerFrame = frame.clone();
        std::thread worker(&MotionDetector::runCallback, this, triggerFrame);

        std::lock_guard<std::mutex> lock(callbackMutex_);
        workers_.push_back(std::move(worker));
    }

    void waitForCallbacks() {
        std::vector<std::thread> temp;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            temp.swap(workers_);
        }
        for (auto& t : temp)
            if (t.joinable()) t.join();
    }

private:
    void runCallback(cv::Mat frame) {
        try { cb_(frame); }
        catch (const std::exception& e) {
            std::cerr << "[motion] event-processing error: " << e.what() << "\n";
        }
    }

    int threshold_;
    double minFrac_, rearmDelay_;
    Callback cb_;
    cv::Mat prev_;
    std::chrono::steady_clock::time_point rearmAt_{};
    std::mutex mutex_, callbackMutex_;
    std::vector<std::thread> workers_;
};

static std::pair<CDeviceInfo,CDeviceInfo> findCameras() {
    CTlFactory& f = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    f.EnumerateDevices(devices);

    bool foundA=false, foundB=false;
    for (const auto& d : devices) {
        std::string s = d.GetSerialNumber().c_str();
        std::cout << "Found camera: " << s << "\n";
        if (s==CAMERA_A_SERIAL) foundA=true;
        if (s==CAMERA_B_SERIAL) foundB=true;
    }

    if (CAMERA_A_SERIAL==CAMERA_B_SERIAL)
        throw std::runtime_error("Camera serials must differ");
    if (!foundA) throw std::runtime_error("Camera A not found, serial="+CAMERA_A_SERIAL);
    if (!foundB) throw std::runtime_error("Camera B not found, serial="+CAMERA_B_SERIAL);

    CDeviceInfo a,b;
    a.SetSerialNumber(CAMERA_A_SERIAL.c_str());
    b.SetSerialNumber(CAMERA_B_SERIAL.c_str());
    return {a,b};
}

int main() {
    PylonInitialize();

    try {
        auto [infoA,infoB] = findCameras();

        CameraStream camB(infoB,FPS,EXPOSURE,CAM_B_HEIGHT,"CamB(coordinate)");
        CoordinateLogger logger(COORD_CSV_PATH);

        int eventId=0;
        std::mutex eventMutex;

        auto handleMotion = [&](const cv::Mat& triggerA) {
            int currentEvent;
            {
                std::lock_guard<std::mutex> lock(eventMutex);
                currentEvent = ++eventId;
            }

            cv::Mat frameA = triggerA.clone();
            cv::Mat frameB = camB.getLatest();

            savePair(frameA,frameB,currentEvent);

            std::optional<ShuttlecockDetection> shcock;
            if (!frameB.empty())
                shcock = detectShuttlecock(frameB).shcock;

            logger.log(currentEvent,shcock);
        };

        MotionDetector detector(
            MOTION_THRESHOLD,
            MOTION_MIN_AREA_FRAC,
            REARM_DELAY_S,
            handleMotion
        );

        // Destroy the trigger stream before the detector, including on exceptions.
        CameraStream camA(infoA,FPS,EXPOSURE,CAM_A_HEIGHT,"CamA(motion)");
        camA.setOnFrame([&](const cv::Mat& frame){ detector.process(frame); });

        camA.start();
        camB.start();

        while (true) {
            auto t0 = std::chrono::steady_clock::now();

            cv::Mat a = camA.getDisplay();
            cv::Mat b = camB.getDisplay();

            if (!a.empty())
                cv::imshow("Camera A (motion-based trigger)", a);

            if (!b.empty()) {
                cv::Mat display;
                cv::cvtColor(b, display, cv::COLOR_GRAY2BGR);

                DetectionResult r = detectShuttlecock(b);

                overlayMask(display, r.whiteMask, cv::Scalar(0,255,0), WHITE_CANDIDATE_ALPHA);
                overlayMask(display, r.blackMask, cv::Scalar(0,50,255), BLACK_CANDIDATE_ALPHA);

                if (r.shcock) {
                    auto& s = *r.shcock;
                    cv::circle(display, {(int)s.whiteX,(int)s.whiteY},
                               (int)s.whiteRadius, {255,255,0}, 2);
                    cv::circle(display, {(int)s.whiteX,(int)s.whiteY},
                               3, {255,255,0}, -1);

                    if (s.blackX && s.blackY && s.blackRadius)
                        cv::circle(display, {(int)*s.blackX,(int)*s.blackY},
                                   (int)*s.blackRadius, {255,0,255}, 2);
                }

                cv::imshow("Camera B (coordinate detection)", display);
            }

            int key = cv::waitKey(1) & 0xFF;
            if (key=='q' || key==27) break;

            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now()-t0).count();
            if (DISPLAY_INT > elapsed)
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(DISPLAY_INT-elapsed));
        }

        camA.stop();
        camB.stop();
        detector.waitForCallbacks();
        cv::destroyAllWindows();
    }
    catch (const GenericException& e) {
        std::cerr << "Pylon exception: " << e.GetDescription() << "\n";
        PylonTerminate();
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        PylonTerminate();
        return 1;
    }

    PylonTerminate();
    return 0;
}
