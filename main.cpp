// Include files to use the pylon API.
#include "PatternGenerator.hpp"
#include "Queue.hpp"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <queue>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>

#define WIDTH 3840
#define HEIGHT 2160
#define FPS 25
#define EXPOSURE 5 //ms
#define MAX_BUFFER_COUNT 30
#define GPIO_PIN 120 //default cam1 line1 for agx and nx

// Namespace for time measurement
using namespace std::chrono;
// Namespace for using pylon objects.
using namespace Pylon;

static const size_t maxCamerasToUse = 3;
std::atomic<bool> is_stop(false);

class ImageGrabEventHandler : public Pylon::CImageEventHandler
{
   public:
    ImageGrabEventHandler(Queue<Pylon::CGrabResultPtr> *t_que) : m_que(t_que) 
    {
        std::cout << "Handler created." << std::endl;
    }

    ImageGrabEventHandler(const ImageGrabEventHandler &) = delete;
    ImageGrabEventHandler &operator=(const ImageGrabEventHandler &) = delete;
    ~ImageGrabEventHandler() = default;

    virtual void OnImageGrabbed(Pylon::CInstantCamera &camera,
                                const Pylon::CGrabResultPtr &ptrGrabResult);

   private:
    Queue<Pylon::CGrabResultPtr> *m_que;
};

void ImageGrabEventHandler::OnImageGrabbed(Pylon::CInstantCamera &camera,
                                           const Pylon::CGrabResultPtr &ptrGrabResult)
{
    intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
    if (ptrGrabResult->GrabSucceeded()){
        m_que->push(ptrGrabResult);
        // measure time for just the first camera
        if(cameraContextValue == 0){
            static auto last_frame = high_resolution_clock::now();
            auto this_frame = high_resolution_clock::now();
            auto frame_duration = duration_cast<milliseconds>(this_frame - last_frame);
            std::cout << "Image grab period: " << frame_duration.count() << std::endl;
            last_frame = this_frame;
        }
    }
}

void ConvertImages(Queue<Pylon::CGrabResultPtr> *t_in_que, Queue<cv::Mat> *t_out_que)
{
    while (!is_stop || !(t_in_que->empty()))
    {
        Pylon::CGrabResultPtr bsl_image; 
        if(t_in_que->tryPop(bsl_image)){
            cv::Mat yuv_image = cv::Mat(bsl_image->GetHeight(), bsl_image->GetWidth(), CV_8UC2,
                                bsl_image->GetBuffer());
            cv::Mat rgb_image;
            cv::cvtColor(yuv_image, rgb_image, cv::COLOR_YUV2BGR_UYVY);
            t_out_que->push(rgb_image);
        }
    }
}

void ProcessImages(std::vector<Queue<cv::Mat>> *t_que, int t_num_cam)
{
    while (!is_stop || !(*t_que)[0].empty())
    {
        static int counter = 0;
        cv::Mat result_image(HEIGHT, t_num_cam * WIDTH, CV_8UC3);
        for (size_t i = 0; i < t_num_cam; i++)
        {
            cv::Mat rgb_image = (*t_que)[i].pop();
            rgb_image.copyTo(result_image(cv::Rect(i * rgb_image.cols, 0, rgb_image.cols, rgb_image.rows)));
        }
        std::string filename = "result" + std::to_string(counter) + ".png";
        cv::imwrite(filename, result_image);
        counter++;
    }
}

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();
    int num_cam = 0;

    auto signal_generator = std::make_shared<PatternGenerator>();
    std::thread siggen_thread;

    try
    {
        // Get the transport layer factory.
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        DeviceInfoList_t devices;
        if ( tlFactory.EnumerateDevices(devices) == 0 )
        {
            throw RUNTIME_EXCEPTION( "No camera present.");
        }

        for ( size_t i = 0; i < devices.size(); ++i)
            std::cout << "Detected Device " << i << ": " << devices[i].GetDeviceID() << std::endl;
        num_cam = std::min(devices.size(), maxCamerasToUse);
        std::cout << num_cam << " cameras will be used." << std::endl;
        // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
        CBaslerUniversalInstantCameraArray cameras(num_cam);

        std::vector<ImageGrabEventHandler *> image_handlers(num_cam);
        std::vector<Queue<Pylon::CGrabResultPtr>> bsl_queues(num_cam);
        std::vector<Queue<cv::Mat>> cv_queues(num_cam);
        std::vector<std::thread> convert_threads(num_cam);

        // Create and attach all Pylon Devices.
        for ( size_t i = 0; i < num_cam; ++i){
            cameras[i].Attach( tlFactory.CreateDevice(devices[i]));
            image_handlers[i] = new ImageGrabEventHandler{&bsl_queues[i]};
            cameras[i].RegisterImageEventHandler(image_handlers[i],
                                                   Pylon::RegistrationMode_ReplaceAll,
                                                   Pylon::Cleanup_Delete);

            // Print the model name of the camera.
            std::cout << "Using device " << cameras[i].GetDeviceInfo().GetModelName() << std::endl;

            cameras[i].Open();
            cameras[i].ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAutoEnums::
                                                                                            ExposureAuto_Off);
            cameras[i].ExposureMode.SetValue(Basler_UniversalCameraParams::ExposureMode_Timed);
            cameras[i].ExposureTime.SetValue(EXPOSURE * 1000);
            cameras[i].Width.SetValue(WIDTH);
            cameras[i].Height.SetValue(HEIGHT);
            cameras[i].BslCenterX.Execute();
            cameras[i].BslCenterY.Execute();
            cameras[i].MaxNumBuffer.SetValue(MAX_BUFFER_COUNT);
            cameras[i].MaxNumQueuedBuffer.SetValue(MAX_BUFFER_COUNT);
            if (GPIO_PIN > 0) {
                // disable limitting frame rate
                cameras[i].AcquisitionFrameRateEnable.SetValue(false);

                // Configure sync triggering
                cameras[i].TriggerSelector.SetValue(Basler_UniversalCameraParams::TriggerSelector_FrameStart);
                cameras[i].TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);
                cameras[i].TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_PeriodicSignal1);
                // cameras[i].TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Line1);
                // cameras[i].TriggerActivation.SetValue(Basler_UniversalCameraParams::TriggerActivation_LevelHigh);
                // Periodic signal settings
                // cameras[i].BslPeriodicSignalSelector.SetValue(
                //                                           Basler_UniversalCameraParams::BslPeriodicSignalSelector_PeriodicSignal1);
                // cameras[i].BslPeriodicSignalDelay.SetValue(0.00);
                // cameras[i].BslPeriodicSignalSource.SetValue(
                //                                           Basler_UniversalCameraParams::BslPeriodicSignalSource_Line1);
                // cameras[i].BslPeriodicSignalActivation.SetValue(
                //                                           Basler_UniversalCameraParams::BslPeriodicSignalActivation_RisingEdge);
            } else {
                // enable limitting frame rate
                cameras[i].AcquisitionFrameRateEnable.SetValue(true);
                cameras[i].AcquisitionFrameRate.SetValue(FPS);
            }

            convert_threads[i] = std::thread(&ConvertImages,  &bsl_queues[i], &cv_queues[i]);

        }

        // Starts grabbing for all cameras starting with index 0. The grabbing
        // is started for one camera after the other. That's why the images of all
        // cameras are not taken at the same time.
        // However, a hardware trigger setup can be used to cause all cameras to grab images synchronously.
        // According to their default configuration, the cameras are
        // set up for free-running continuous acquisition.
        cameras.StartGrabbing(Pylon::GrabStrategy_OneByOne,
                        Pylon::GrabLoop_ProvidedByInstantCamera);

        //start image processing thread
        std::thread processing_thread(&ProcessImages, &cv_queues, num_cam);

        // start hw trigggering signal
        if(GPIO_PIN > 0){
            try {
                double period = 1.0 / FPS;
                double duty = period / 4.0;  // 1/4 of the square wave will be high and rest will be low
                if (signal_generator->open(GPIO_PIN, period, duty)) {
                    signal_generator->generateSquareWave();
                    std::cout << "Hardware triggering is configured as; period: " << period << ", duty cycle: " << duty << std::endl;
                    siggen_thread = std::thread(&PatternGenerator::loopAllPatternsRealtime,
                                                    signal_generator);
                }
            } catch (const std::exception &e) {
                std::cerr << "HW triggering failed to start: " << e.what() << '\n';
            }        
        }

        // Comment the following two lines to disable waiting on exit.
        std::cout << std::endl << "Press enter to exit." << std::endl;
        while( std::cin.get() != '\n');

        std::cout << "Stopping image grabbing." << std::endl;
        cameras.StopGrabbing();

        std::cout << "Stopping hw trigger signal." << std::endl;
        signal_generator->stopAllPatterns();
        if(siggen_thread.joinable()) siggen_thread.join();

        is_stop = true;

        std::cout << "Processing images in the que." << std::endl;
        if(processing_thread.joinable()) processing_thread.join();
        for (size_t i = 0; i < num_cam; i++)
        {
            if(convert_threads[i].joinable()) 
                convert_threads[i].join();
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        std::cerr << "An exception occurred." << std::endl
        << e.GetDescription() << std::endl;
        exitCode = 1;
    }

    std::cout << "Closing the app!!!" << std::endl;
    
    // Releases all pylon resources.
    PylonTerminate();
        
    return exitCode;
}