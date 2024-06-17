extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/time.h>
}

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    av_register_all();
    avformat_network_init();

    AVFormatContext* pFormatCtx = nullptr;

    if (avformat_open_input(&pFormatCtx, "url_consume", NULL, NULL) != 0) {
        std::cerr << "Couldn't open source." << std::endl;
        return -1;
    }

    if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
        std::cerr << "Couldn't find stream information." << std::endl;
        return -1;
    }

    int videoStream = av_find_best_stream(pFormatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (videoStream < 0) {
        std::cerr << "No video stream found." << std::endl;
        return -1;
    }

    AVCodecContext* pCodecCtx = pFormatCtx->streams[videoStream]->codec;
    AVCodec* pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
    if (!pCodec) {
        std::cerr << "Codec not found." << std::endl;
        return -1;
    }

    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        std::cerr << "Could not open codec." << std::endl;
        return -1;
    }

    int frameFinished;
    AVFrame* pFrame = av_frame_alloc();
    AVFrame* pFrameRGB = av_frame_alloc();
    AVPacket packet;
    struct SwsContext* sws_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, NULL, NULL, NULL);
    cv::Mat img(pCodecCtx->height, pCodecCtx->width, CV_8UC3);
    cv::Mat gray, frameDelta, thresh, lastFrame; 
    std::vector<std::vector<cv::Point>> cnts;

    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer, AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height, 1);

    int frameNumber(1);
    auto last(av_gettime());
    while (av_read_frame(pFormatCtx, &packet) >= 0) {
        if (packet.stream_index == videoStream) {
            avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);
            if (frameFinished) {
                sws_scale(sws_ctx, (uint8_t const* const*)pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);
                img.data = pFrameRGB->data[0];
                // ...

                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
                cv::GaussianBlur(gray, gray, cv::Size(25, 25), 0);  // Aumentando o núcleo

                if (!lastFrame.empty()) {
                    cv::absdiff(lastFrame, gray, frameDelta);
                    cv::threshold(frameDelta, thresh, 10, 300, cv::THRESH_BINARY);  // Ajustando o limite

                    // Aplicando operações morfológicas
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
                    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);
                    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, kernel);

                    cv::dilate(thresh, thresh, cv::Mat(), cv::Point(-1, -1), 2);
                    cv::findContours(thresh, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                    // for (auto &c : cnts) {
                    //     if (cv::contourArea(c) < 600)  // Aumentando a área mínima
                    //         continue;
                    //     cv::Rect box = cv::boundingRect(c);
                    //     cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);
                    // }

                    bool movementDetected = false;
                    for (auto &c : cnts) {
                        if (cv::contourArea(c) < 1000)
                            continue;

                        movementDetected = true;
                        cv::Rect box = cv::boundingRect(c);
                        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);
                    }

                    // If movement detected, save the frame
                    if (movementDetected && ((av_gettime() - last) / 1000000) > 1) {
                        std::string filename = "movement_detected_" + std::to_string(frameNumber) + ".jpg";
                        cv::imwrite(filename, img);
                        last = av_gettime();
                        frameNumber++;
                    }
                }

                cv::imshow("Frame", img);
                // cv::imshow("Gray", gray);
                gray.copyTo(lastFrame);
                if(cv::waitKey(1) == 'q')
                    break;
            }
        }
        av_free_packet(&packet);
    }

    av_free(pFrameRGB);
    av_free(pFrame);
    avcodec_close(pCodecCtx);
    avformat_close_input(&pFormatCtx);
    return 0;
}