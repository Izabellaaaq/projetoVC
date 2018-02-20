#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
const int KEY_SPACE = 32;
const int KEY_ESC = 27;
cv::CascadeClassifier cascade;

std::vector<cv::Rect> rectMax(const std::vector<cv::Rect> &vRects)
{
	std::vector<cv::Rect> v_r;
	cv::Rect r = vRects[0];
	int index_max_rect = 0;

	for (int i = 1; i < (int) vRects.size(); ++i)
	{
		if (vRects[i].area() > r.area())
		{
			r = vRects[i];
			index_max_rect = i;
		}
	}

	for(int i = 0; i < (int) vRects.size(); ++i)
	{
		if(i == index_max_rect)continue;
		v_r.push_back(vRects[i]);
	}
	return v_r;
}

bool exist_intersections(std::vector<cv::Rect> v_rects)
{
	bool res = false;

	for(int i = 0; i < (int) v_rects.size(); ++i)
	{
		for(int j = i+1; j < (int) v_rects.size(); ++j)
		{
			cv::Rect r = v_rects[i] & v_rects[j];
			if (r.area() > 0)
			{
				return true;
			}
		}
	}

	return res;
}

std::vector<cv::Rect> remove_intersections_aux(const std::vector<cv::Rect> &vRects)
{
	std::vector<cv::Rect> v_out;
	std::vector<int> v_rects_used (vRects.size(), 0);

	for(int i = 0; i < (int) vRects.size(); ++i)
	{
		if(v_rects_used[i] == 1) continue;
		cv::Rect r1 = vRects[i];
		std::vector<cv::Rect> v_inters;
		for(int j = i+1; j < (int) vRects.size(); ++j)
		{
			if(v_rects_used[j] == 1) continue;
			cv::Rect r2 = vRects[j];
			cv::Rect inter = r1 & r2;
			cv::Rect min_rect = r1.area() < r2.area()? r1 : r2;

//			std::cout<<"("<<inter.area()<<" != 0 && ("<<inter.area() <<") >= "<<min_rect.area() * 0.5<<")"<<std::endl;
			if (inter.area() != 0 && (inter.area() >= (min_rect.area() * 0.5)))
			{
				v_rects_used[j] = 1;
				v_rects_used[i] = 1;

				v_inters.push_back(r1);
				v_inters.push_back(r2);
			}
		}

		if(exist_intersections(v_inters))
		{
			std::cout<<"remove again..."<<std::endl;
			getchar();
			v_inters = remove_intersections_aux(v_inters);
			std::cout<<"chega aqui??"<<std::endl;
			for(int ii = 0; ii < (int) v_inters.size(); ++ii)
			{
				v_out.push_back(v_inters[ii]);
			}
		}
		else if (v_inters.size() == 0)
		{
			v_out.push_back(r1);
		}
		else
		{
			for(int ii = 0; ii < (int) v_inters.size(); ++ii)
			{
				v_out.push_back(v_inters[ii]);
			}
		}
	}

	return v_out;
}

std::vector<cv::Rect> removeIntersections(const std::vector<cv::Rect> &vRects)
{
	std::vector<cv::Rect> vRects_aux;
	std::vector<int> isRectUsed(vRects.size(), 0);

	for (int i = 0; i < (int)vRects.size(); ++i)
	{
		bool flag = true;
		cv::Rect r = vRects[i];
		std::vector<cv::Rect> interRects;
		for (int j = i + 1; j < (int) vRects.size(); ++j)
		{
			cv::Rect inter = vRects[i] & vRects[j];
			double min_area = vRects[i].area() < vRects[j].area()? vRects[i].area() : vRects[j].area();
			if (inter.area() != 0 && inter.area() >= min_area*0.7)
			{
				interRects.push_back(vRects[j]);
				flag = false;
				isRectUsed[j] = 1;
			}
		}

		if (!flag)
		{
			interRects.push_back(r);
			std::vector<cv::Rect> v_r = rectMax(interRects);
			for(int ii = 0; ii < (int)v_r.size(); ++ii)
			{
				vRects_aux.push_back(v_r[ii]);
			}
		}
		else if (isRectUsed[i] == 0)
		{

			vRects_aux.push_back(r);
		}
	}

	return vRects_aux;
}

void removeRects(std::vector<cv::Rect> &vRects)
{
	std::vector<int> n;

	n.push_back(vRects.size());


	int index = 0;
	do{
		vRects = removeIntersections(vRects);
		++index;
		n.push_back(vRects.size());
	}while(n[index] != n[index - 1]);

}


std::vector<cv::Rect> new_cars(std::vector<cv::Rect> old_detections, std::vector<cv::Rect> new_detections, int count)
{
	std::vector<cv::Rect> v;

	for(int i = 0; i < (int)new_detections.size(); ++i)
	{

		for(int j = 0; j < (int) old_detections.size(); ++j)
		{
			cv::Rect inter = new_detections[i] & old_detections[j];
			double min_area = new_detections[i].area() < old_detections[j].area()? new_detections[i].area() : old_detections[j].area();
			if(inter.area() /  min_area <= 0.5 )
			{
				++count;
				v.push_back(new_detections[i]);
			}

		}
	}

	return v;
}

std::vector<std::pair<cv::Rect, int> > count_cars(std::vector<cv::Rect> v_new,std::vector<std::pair<cv::Rect, int> > v_old, int &count)
{
	std::vector<std::pair<cv::Rect, int> > res;

	if (v_old.size() == 0)
	{
		for(int j = 0; j < (int)v_new.size(); ++j)
		{
			std::pair<cv::Rect, int> aux;
			aux.first = v_new[j];
			aux.second = ++count;
			res.push_back(aux);
		}
	}
	else
	{
		for(int j = 0; j < (int)v_new.size(); ++j)
		{
			bool flag = false;
			for(int i = 0; i < (int) v_old.size() && !flag; ++i)
			{
				cv::Rect inter = v_old[i].first & v_new[j];
				if(inter.area() >= v_old[i].first.area() * 0.5)
				{
					v_old[i].first = v_new[j];
					res.push_back(v_old[i]);
					flag = true;
				}
			}
			if(!flag)
			{
				std::pair<cv::Rect, int> aux;
				aux.first = v_new[j];
				aux.second = ++count;
				res.push_back(aux);
			}
		}
	}


	return res;
}

std::string int2string(int n)
{
	std::stringstream ss;
	ss << n;
	return ss.str();
}

cv::Mat applySolbelFilter(cv::Mat img)
{
	cv::Mat gradX, gradY;
	cv::Sobel( img, gradX, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
	cv::Sobel( img, gradY, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

	int threshold = 50;

	gradX = gradX > threshold;
	gradY = gradY > threshold;

	cv::Mat grad = gradX + gradY;

	cv::morphologyEx(grad, grad, cv::MORPH_DILATE, cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*3 + 1, 2*3+1 ), cv::Point( 3, 3) ), cv::Point(-1,-1), 1, cv::BORDER_DEFAULT);

	return grad;
}
std::vector<cv::Rect> detect_from_mov(cv::Mat mov)
{

//	cv::Mat sobel = applySolbelFilter(mov);
	std::vector<cv::Rect> vRects;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours( mov, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	std::vector<std::vector<cv::Point> > contours_poly( contours.size() );


	for(int i = 0; i < contours.size(); ++i)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], cv::arcLength(cv::Mat(contours[i]), true) * 0.01, true);
		cv::Rect rect =cv::boundingRect( cv::Mat(contours_poly[i]) );
		cv::Scalar color = cv::Scalar( 0, 255, 0 );

		vRects.push_back(rect);
	}

	return vRects;
}

std::vector<cv::Rect> detect_cars(cv::Mat frame, std::string m)
{
	std::vector<cv::Rect> bbox;

	if(m == "cascade")
	{
		cascade.detectMultiScale(frame, bbox, 1.1, 1, 0, cv::Size(0,0));
	}

	return bbox;
}

int main(int argc, char** argv)
{

	std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;

	cv::VideoCapture cap;
	cv::Mat frame;

	if(argc < 2)
	{
		std::cout << "Usage " << argv[0] << " video.avi" << std::endl;
		return 0;
	}

	cascade.load("cars.xml");
	cap.open(argv[1]);
	cv::namedWindow("video", 1);
	int key = 0;
	std::vector<std::pair<cv::Rect, int> > old_detect;
	int n_cars = 0;
	cv::Mat back;
	do
	{
		cap >> frame;

		if(back.empty())
		{
			frame.copyTo(back);
			continue;
		}
//		cv::Mat diff = back - frame;
//		cv::cvtColor(frame, frame, CV_RGB2GRAY);
//		cv::cvtColor(diff, diff, CV_RGB2GRAY);
//		cv::morphologyEx(grad, grad, cv::MORPH_DILATE, cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*3 + 1, 2*3+1 ), cv::Point( 3, 3) ), cv::Point(-1,-1), 1, cv::BORDER_DEFAULT);
//		cv::Mat mov = diff > 80;
//		cv::threshold(diff, diff, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//		diff.convertTo(mov, CV_8UC1);
//		std::vector<cv::Rect> bbox_cars = detect_from_mov(diff);
//		for(int i = 0; i < (int) bbox_cars.size(); ++i)
//		{
//			cv::rectangle(frame, bbox_cars[i].tl(), bbox_cars[i].br(), cv::Scalar(255, 0, 0),4, 2);
//		}
//		frame.copyTo(back);
//		cv::imshow("diff", diff);
////		cv::imshow("mov", mov);
//		cv::imshow("frame", frame);
//		cv::waitKey(-1);
//		continue;
		if(frame.empty()) break;

		std::vector<cv::Rect> bbox = detect_cars(frame, "cascade");

		removeRects(bbox);
		old_detect = count_cars(bbox, old_detect, n_cars);
		for (int i = 0 ; i < (int) old_detect.size(); ++i)
		{
			cv::rectangle(frame, old_detect[i].first.tl(), old_detect[i].first.br(), cv::Scalar(255,0,0), 8, 2);
			cv::putText(frame, int2string(old_detect[i].second),old_detect[i].first.tl(),cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 4, 2);
		}
		cv::imshow("video", frame);
		key = cvWaitKey(-1);

	}while(1);

	std::cout<<"total cars: "<<n_cars<<std::endl;

	return 0;
}
