#include "opencv2/opencv.hpp"

cv::Mat get_region_of_interest(cv::Mat img_edges, cv::Point *points);
std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> divide_left_right(std::vector<cv::Vec4f> lines);
void drawRectangle(cv::Mat& img, float lpos, float rpos);
void drawLines(cv::Mat& img, std::vector<cv::Vec4f> lines, cv::Scalar color);
void histogram_stretching_mod(const cv::Mat& src, cv::Mat& dst);

int main()
{
    cv::VideoCapture cap("../Sub_project.avi");
    if (!cap.isOpened()) {
        std::cerr << "Video open failed!" << std::endl;
        return -1;
    }

    cv::VideoWriter output("output.mp4",
            cv::VideoWriter::fourcc('X', '2', '6', '4'),
            cap.get(cv::CAP_PROP_FPS),
            cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap.get(cv::CAP_PROP_FRAME_HEIGHT)));

    cv::Mat frame, frame_gray, edge, houghFrame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Empty frame!" << std::endl;
            break;
        }

        // 원본 영상 그레이스케일 변환
        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // 히스토그램 스트레칭
        histogram_stretching_mod(frame_gray, frame_gray);

        // 가우시안 블러
        GaussianBlur(frame_gray, frame_gray, cv::Size(), 1.5);

        // 캐니 에지
        Canny(frame_gray, edge, 50, 150);
        cv::Mat mask = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8UC1);
        for (int i = 360; i < frame_gray.rows; ++i) {
            uchar* ptr = mask.ptr<uchar>(i);
            for (int j = 0; j < frame_gray.cols; ++j) {
                if((j > 220) && (j < 420)){
                    continue;
                }
                ptr[j] = 255;
            }
        }

        // 허프 변환
        cv::Mat dst;
        copyTo(edge, dst, mask);
        std::vector<cv::Vec4f> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 35, 30, 3);
        std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> pair_lines = divide_left_right(lines);
        std::vector<cv::Vec4f> line_L=pair_lines.first;
        std::vector<cv::Vec4f> line_R=pair_lines.second;
        std::vector<cv::Vec4f> new_lines;

        // 인식한 차선 그리기
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 2, cv::LINE_AA);
        }

        // lpos, rpos 구하기
        int frame_number = cap.get(cv::CAP_PROP_POS_FRAMES);
        float lpos=0;
        float rpos=0;
        if(line_L.size() > 0){
            for(size_t i = 0; i < line_L.size(); i++){
                cv::Vec4f element = line_L[i];
                float x1 = element[0];
                float y1 = element[1];
                float x2 = element[2];
                float y2 = element[3];
                float slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
                lpos += (400 - y1 + slope * x1) / slope ;
            }
            lpos = lpos/line_L.size();
        }
        else{
            lpos=0;
        }
        if(line_R.size() > 0){
            for(size_t i = 0; i < line_R.size(); i++){
                cv::Vec4f element = line_R[i];
                float x1 = element[0];
                float y1 = element[1];
                float x2 = element[2];
                float y2 = element[3];
                float slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
                rpos += (400 - y1 + slope * x1) / slope ;
            }
            rpos = rpos/line_R.size();
        }
        else{
            rpos=640;
        }

        // offset 선 그리기
        line(frame, cv::Point(0, 400), cv::Point(frame.cols, 400), cv::Scalar(255, 255, 255), 2);

        // lpos, rpos 좌표에 사각형 그리기
        drawRectangle(frame, lpos, rpos);
         if (frame_number % 30 == 0) {
            std::cout << lpos << ',' << rpos << std::endl;
            // ofs << lpos << "," << rpos << endl;
        }

        // lpos, rpos 좌표에 좌표값 그리기
        cv::putText(frame, cv::format("(%d)", (int)lpos),
		cv::Point(lpos + 20, 400.0),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::putText(frame, cv::format("(%d)", (int)rpos),
		cv::Point(rpos - 70, 400.0),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

        imshow("frame", frame);
        if (cv::waitKey(1000 / cap.get(cv::CAP_PROP_FPS)) == 27){
            break;
        }

        output << frame;

    }
    cap.release();
    cv::destroyAllWindows();
}

std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> divide_left_right(std::vector<cv::Vec4f> lines)
{
	float low_slope_threshold = 0;
	int32_t high_slope_threshold = 10;
	int32_t width_threshold = 90;
	std::vector<float> slopes;
	std::vector<cv::Vec4f> new_lines;
	int32_t x1, y1, x2, y2;
	float slope, abs_slope;
	for (cv::Vec4f line: lines) {
		x1 = line[0];
		y1 = line[1];
		x2 = line[2];
		y2 = line[3];
		if (x2 - x1 == 0)
		{
			slope = 0;
		}
		else
		{
			slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
		}
		abs_slope = abs(slope);
		if ((abs_slope > low_slope_threshold) && (abs_slope < high_slope_threshold)) {
			slopes.push_back(slope);
			new_lines.push_back(line);
		}
	};
	std::vector<cv::Vec4f> left_lines, right_lines;
	cv::Vec4i line;
	for (size_t i = 0; i < slopes.size(); i++) {
		line = new_lines[i];
		slope = slopes[i];
		x1 = line[0];
		y1 = line[1];
		x2 = line[2];
		y2 = line[3];
		if ((slope < 0) && (x2 < 320 - width_threshold) && (x1 < 320 - width_threshold))
		{
			left_lines.push_back(line);
		}
		else if ((slope > 0) && (x1 > 320 + width_threshold) && (x2 > 320 + width_threshold))
		{
			right_lines.push_back(line);
		}
	}
	std::pair<std::vector<cv::Vec4f>, std::vector<cv::Vec4f>> all_lines;
	all_lines.first = left_lines;
	all_lines.second = right_lines;
	return all_lines;
}

void drawRectangle(cv::Mat& img, float lpos, float rpos)
{
    float gOffset = 400;
	float f_center = (lpos + rpos) / 2;
	f_center = round(f_center * 100) / 100;
	int32_t center = static_cast<int>(f_center);
	int32_t y1 = 5 + gOffset;
	int32_t y2 = gOffset-5;
	cv::rectangle(img, cv::Point(lpos - 5, y1), cv::Point(lpos + 5, y2), cv::Scalar(0, 0, 255), 2);
	cv::rectangle(img, cv::Point(rpos - 5, y1), cv::Point(rpos + 5, y2), cv::Scalar(0, 0, 255), 2);
}

void histogram_stretching_mod(const cv::Mat& src, cv::Mat& dst)
{
	int hist[256] = { 0, };
	for (int y = 0; y < src.rows; y++) {
		for (int x = 0; x < src.cols; x++) {
			hist[src.at<uchar>(y, x)]++;
		}
	}
	int gmin = 255, gmax = 0;
	int ratio = int(src.cols * src.rows * 0.01);
	for (int i = 0, s = 0; i < 255; i++) {
		s += hist[i];
		if (s > ratio) {
			gmin = i;
			break;
		}
	}
	for (int i = 255, s = 0; i >= 0; i--) {
		s += hist[i];
		if (s > ratio) {
			gmax = i;
			break;
		}
	}
    normalize(src, dst, gmin, gmax, cv::NORM_MINMAX);
}
