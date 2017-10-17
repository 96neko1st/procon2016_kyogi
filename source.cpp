#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include<random>
#include<string>
#include<strstream>
#include <math.h>
#define PI 3.14159
#define DATABASE 100
#define a 0.3

using namespace std;
using namespace cv;


//**		ピースの情報を格納する配列		**//
typedef struct piece {
	int piece_inf;					//何角形かの情報
	int Intersection_x[32];			//ピースの交点xを格納
	int Intersection_y[32];			//ピースの交点yを格納
	double piece_angle[32];			//ピースの角度を格納
	double piece_length[32];		//ピースの長さを格納
}piece;

piece piece_non[DATABASE];


//**		枠の情報を格納する配列		**//
typedef struct FRAME {
	int piece_inf;					//何角形かの情報
	int Intersection_x[DATABASE];	//枠の交点xを格納
	int Intersection_y[DATABASE];	//枠の交点yを格納
	double angle[DATABASE];			//枠の角度を格納
	double length[DATABASE];		//枠の長さを格納
}frame;

frame frame_inf[DATABASE];


//再帰処理に使用
int *p;


//線分交差再帰処理(ループの深さ)
int n = 1;


//画像カウント
int Auto_img_count = 0;


int Manual_img_count = 0;


//交点の数を保存
int coner_count = 3;


//使用する画像の番号
int img_number;


//ピースの全体画像をリサイズするためのflag
int change_flag = 0;


//使用するコマンドの値
int command = 0;


//x交点保存(ソートされる前)
int piece_x[DATABASE][DATABASE];


//y交点保存(ソートされる前)
int piece_y[DATABASE][DATABASE];


//手動で認識する画像保存
Mat src[32];


//自動で認識する画像保存
Mat sample[32];


//枠連結前画像保存
Mat src_img[2];


//枠連結後画像保存
Mat dst_img;


//枠の元画像を保存
Mat frame_src;


//ROIで1つの枠の領域を抽出(元画像)
Mat result_img[DATABASE];


//ROIで1つの枠の領域を抽出(白黒画像)
Mat frame_dst[DATABASE];


//ROIで1つのピースを抽出(元画像)
Mat Piece_ROI[DATABASE];


//ROIで1つのピースを抽出(色付き)
Mat dst[DATABASE];


//ROIで1つのピースを抽出(白黒画像)
Mat bin_img[DATABASE];


//三点の座標ABCのBCのx座標の差分
double BC_x = 0;


//三点の座標ABCのBAのx座標の差分
double BA_x = 0;


//三点の座標ABCのBCのy座標の差分
double BC_y = 0;


//三点の座標ABCのBAのy座標の差分
double BA_y = 0;


//角度の合計
double num = 0;


//ピースの個数カウント
int piece_cnt = 0;


//枠の個数カウント
int frame_cnt = 0;


//交点の数カウント
int counter = 0;


//輪郭追跡実行回数記憶カウント(枠)
int frame_count_chase = 0;


//角度取得実行回数カウント(枠)
int frame_count_angle = 0;


//枠表示実行回数カウント
int frame_count_print = 0;


//交点線分描画カウント
int Auto_count = 0;


//輪郭追跡実行回数記憶カウント(ピース)
int count_chase = 0;


//角度取得実行回数カウント(ピース)
int count_angle = 0;


//ピース表示実行回数カウント
int count_print = 0;


//**入力した輪郭を取得**//
void Get_ManualPiece(Mat value[DATABASE]);


//**取得した画像のピースの輪郭を取得**//
void Get_AutoPiece(Mat value[DATABASE]);


//**ピースの角度や長さを取得**//
void Get_PieceInformation(int val);


//**画像を連結して交点と輪郭を取得**//
void Get_Frame_IntersectionPoint();


//**交点から枠の角度と長さを取得**//
void Get_FrameInformation(int value);


//**ピース配置後の情報を取得**//
void Get_AfterPlacement_Information(Mat value);


//**輪郭追跡時計回りソート**//
void Sort_Piece_IntersectionPoint(int value);


//**輪郭を追跡し交点を時計回りにソート**//
void Sort_Frame_IntersectionPoint(Mat value[DATABASE], int val);


//**ピースの外角の角度を内角に変換する（自動)**//
void Convert_PieceAngle(int start, int end, int depth, int maxdepth, int number);


//**ピースの外角の角度を内角に変換する（手動)**//
void Convert_Cmd_PieceAngel(int val, int val1);


//**枠の外角の角度を内閣に変換する(手動)**//
void Convert_Cmd_FrameAngel(int val, int val1);


//**枠の外角の角度を内角に変換する（自動）**//
void Convert_FrameAngle(int start, int end, int depth, int maxdepth, int number);


//**枠の交点を画像に描画**//
void Drawing_FramePoint();


//**ピースの交点を画像に描画**//
void Drawing_PiecePoint(int val1);


//**ピースの画像を表示**//
void Display_Image();


//**線分交差判定**//
//int Judge_Intersection(int p1x, int p2x, int p3x, int p4x, int p1y, int p2y, int p3y, int p4y);


//**ピースの当たり判定**//
//int Hit_PieceTest(int val);



int main()
{

	p = (int*)malloc(sizeof(int) * 16);

	//ピースの角ごとの画像
	src[6] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src003.jpg");
	//自動化で読み取る画像
	sample[0] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src002.jpg");
	sample[1] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src004.jpg");

	src_img[0] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\c002.jpg");
	src_img[1] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\c001.jpg");

	int flag = 0;

	while (flag == 0) {
		cout << "0:終了 1:手動入力 2:自動入力 3:枠 4:隣り合ったピースを検索 5:逆順検索 6:画像表示" << endl;
		cin >> command;
		switch (command) {
		case 0:	//終了
			if (frame_cnt == 0) { cout << "枠の情報を入れてください" << endl;  break; }
			flag = 1;
			break;
		case 1:	//入力モード
			cout << "何角形ですか？" << endl;
			cin >> img_number;
			if (src[img_number].empty()) { break; }
			Get_ManualPiece(src);
			Manual_img_count++;
			break;
		case 2:	//自動入力モード
			cout << "入力画像を選択してください" << endl;
			cin >> img_number;
			if (sample[img_number].empty()) { break; }
			Get_AutoPiece(sample);
			Auto_img_count++;
			break;
		case 3:	//枠の情報抽出モード
			if (src_img[0].empty() || src_img[1].empty()) { break; }
			Get_Frame_IntersectionPoint();
			imshow("枠", result_img[frame_cnt - 1]);
			waitKey();
			break;
		case 4:	//piece画像表示
			Display_Image();
			break;
		default:
			cout << "選択肢にない番号です" << endl;
			break;
		}
	}

	cout << piece_cnt;
	waitKey();
	return 0;
}



//**ROIの領域を取得し交点の座標を取得**//
void Get_AutoPiece(Mat value[DATABASE]) {
	//読み込み画像をリサイズする
	resize(value[img_number], value[img_number], cv::Size(), a, a);

	//二値化画像保存
	Mat bin;
	cvtColor(value[img_number], bin, CV_BGR2GRAY);
	// 二値化
	threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	//ラべリング処理
	Mat LabelImg;
	Mat stats;
	Mat centroids;

	int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids);


	// ラベリング結果の描画色を決定
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}


	// ラベリング結果の描画
	Mat Dst(value[img_number].size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROIの設定
	for (int i = 1; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int Area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		//面積が100以下の領域は除去
		if (Area > 100) {
			//ROIで1つのピースを抽出(元画像)
			Piece_ROI[piece_cnt] = Mat(value[img_number], Rect(x - 5, y - 5, width + 10, height + 10));

			//ROIで1つのピース抽出(色付き)
			dst[piece_cnt] = Mat(Dst, Rect(x - 5, y - 5, width + 10, height + 10));


			//ROIで1つのピース抽出(白黒画像)
			bin_img[piece_cnt] = Mat(bin, Rect(x - 5, y - 5, width + 10, height + 10));

			//輪郭の座標リスト
			std::vector< std::vector< cv::Point > > contours;

			//輪郭取得
			findContours(bin_img[piece_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			std::vector< cv::Point > approx;

			for (auto contour = contours.begin(); contour != contours.end(); contour++) {
				//抽出した座標を描画
				polylines(dst[piece_cnt], *contour, true, cv::Scalar(0, 255, 255), 1);

				//輪郭を直線近似する
				approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
			}


			//重心にピース番号を表示
			double *param = centroids.ptr<double>(i);
			int x_point = static_cast<int>(param[0]);
			int y_point = static_cast<int>(param[1]);

			stringstream b;
			b << piece_cnt;
			putText(sample[img_number], b.str(), Point(x_point, y_point), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 255), 2);



			//ピースの輪郭数を格納
			coner_count = approx.size();
			cout << "輪郭は=" << coner_count << endl;


			//ピースの交点座標を格納
			for (int j = 0; j < approx.size(); j++) {
				piece_x[piece_cnt][j] = approx[j].x;
				piece_y[piece_cnt][j] = approx[j].y;
			}


			//ピースカウントを増やす
			piece_cnt = piece_cnt + 1;


			Sort_Piece_IntersectionPoint(coner_count);
			Drawing_PiecePoint(coner_count);
			Get_PieceInformation(coner_count);
		}
	}
}



//**入力した輪郭の交点の座標を取得**//
void Get_ManualPiece(Mat value[DATABASE]) {
	Mat bin;
	resize(value[img_number], value[img_number], cv::Size(), a, a);
	//グレイスケール化
	cvtColor(value[img_number], bin, CV_BGR2GRAY);
	// 二値化
	threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//ラべリング処理
	Mat LabelImg;
	Mat stats;
	Mat centroids;
	int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids); //認識した数

	// ラベリング結果の描画色を決定
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	// ラベリング結果の描画
	Mat Dst(value[img_number].size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROIの設定
	for (int i = 2; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		if (area > 100) {//邪魔な領域除去処理
			Piece_ROI[piece_cnt] = Mat(value[img_number], Rect(x - 5, y - 5, width + 10, height + 10));
			dst[piece_cnt] = Mat(Dst, Rect(x - 5, y - 5, width + 10, height + 10));

			bin_img[piece_cnt] = Mat(bin, Rect(x - 5, y - 5, width + 10, height + 10));

			//harrisコーナ―検出
			vector<Point2f> coners;
			goodFeaturesToTrack(bin_img[piece_cnt], coners, img_number, 0.1, 3, Mat(), 3, true);
			vector<Point2f>::iterator it_coners = coners.begin();
			for (; it_coners != coners.end(); ++it_coners) {
				//配列に交点座標を格納
				piece_x[piece_cnt][counter] = it_coners->x;
				piece_y[piece_cnt][counter] = it_coners->y;
				counter += 1;
			}


			//重心にピース番号を表示
			double *param = centroids.ptr<double>(i);
			int x_point = static_cast<int>(param[0]);
			int y_point = static_cast<int>(param[1]);

			stringstream b;
			b << piece_cnt;
			putText(src[img_number], b.str(), Point(x_point, y_point), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 255), 2);

			piece_cnt = piece_cnt + 1;
			counter = 0;
		}
	}
	Sort_Piece_IntersectionPoint(img_number);
	Drawing_PiecePoint(img_number);
	Get_PieceInformation(img_number);
}



//**ピースの角度と長さを求める**//
void Get_PieceInformation(int val) {
	for (int i = count_angle; i < piece_cnt; ++i) {
		cout << i << "番目" << endl;
		for (int j = 0; j <val; j++) {
			//三点の座標から角度を求める
			if (j == val - 1) {
				BC_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[0];
				BA_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[j - 1];
				BC_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[0];
				BA_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[j - 1];
			}
			else if (j == 0) {
				BC_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[j + 1];
				BA_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[val - 1 - j];
				BC_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[j + 1];
				BA_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[val - 1 - j];
			}
			else {
				BC_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[j + 1];
				BA_x = piece_non[i].Intersection_x[j] - piece_non[i].Intersection_x[j - 1];
				BC_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[j + 1];
				BA_y = piece_non[i].Intersection_y[j] - piece_non[i].Intersection_y[j - 1];
			}

			//ピースの角形の情報挿入
			piece_non[i].piece_inf = val;
			//ピースの長さを計算
			piece_non[i].piece_length[j] = sqrt((BC_x)* (BC_x)+(BC_y)* (BC_y));

			cout << "長さ=" << piece_non[i].piece_length[j] << endl;

			//ルートの計算
			double sq_1 = cvSqrt(BC_x*BC_x + BC_y*BC_y);
			double sq_2 = cvSqrt(BA_x*BA_x + BA_y*BA_y);

			double BCA = ((BC_x*BA_x) + (BC_y*BA_y));//分子
			double sq_12 = (sq_1*sq_2);//分母
			//角度の計算
			double ang = BCA / sq_12;
			double angle = acos(ang);

			angle = angle * 180 / PI;
			piece_non[i].piece_angle[j] = angle;
			num = num + angle;
			cout << "角度=" << piece_non[i].piece_angle[j] << endl;
		}


		int flag_1 = 1;
		int all_angle = num;

		if (all_angle != (val - 2) * 180) {
			flag_1 = 0;
		}

		if (flag_1 == 0) {
			Convert_PieceAngle(0, val, n - 1, n - 1, i);
			all_angle = num;
			if (all_angle != (val - 2) * 180) {
				n = 2;
				Convert_PieceAngle(0, val, n - 1, n - 1, i);
				all_angle = num;
				if (all_angle != (val - 2) * 180) {
					n = 3;
					Convert_PieceAngle(0, val, n - 1, n - 1, i);
					all_angle = num;
					if (all_angle != (val - 2) * 180) {
						n = 4;
						Convert_PieceAngle(0, val, n - 1, n - 1, i);
						all_angle = num;
						if (all_angle != (val - 2) * 180) {
							Convert_Cmd_PieceAngel(val, i);
						}
					}
				}
			}
		}

		cout << "合計の角度=" << num << endl;
		n = 1;
		num = 0;
		cout << endl;
		count_angle++;
	}
}



//**画像連結**//
void Get_Frame_IntersectionPoint() {
	resize(src_img[0], src_img[0], cv::Size(), a, a);
	resize(src_img[1], src_img[1], cv::Size(), a, a);

	//画像横連結
	hconcat(src_img, 2, dst_img);
	Mat frame_src = dst_img.clone();
	//グレイスケール化
	cvtColor(dst_img, dst_img, CV_BGR2GRAY);
	//二値化
	threshold(dst_img, dst_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//反転
	dst_img = ~dst_img;

	//ラベリング情報
	Mat LabelImg;
	Mat stats;
	Mat centroids;
	int nLab = cv::connectedComponentsWithStats(dst_img, LabelImg, stats, centroids);

	//ROIの設定
	for (int i = 3; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		if (area > 100) {//邪魔な領域除去処理
			//ROIで領域を取得(元画像)
			result_img[frame_cnt] = Mat(frame_src, Rect(x - 5, y - 5, width + 10, height + 10));

			//ROIで領域を取得(白黒画像)
			frame_dst[frame_cnt] = Mat(dst_img, Rect(x - 5, y - 5, width + 10, height + 10));

			//輪郭の座標リスト
			std::vector< std::vector< cv::Point > > contours;

			//輪郭取得
			findContours(frame_dst[frame_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			std::vector< cv::Point > approx;

			for (auto contour = contours.begin(); contour != contours.end(); contour++) {
				//輪郭を直線近似する
				cv::approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
			}

			coner_count = approx.size();

			cout << "輪郭は=" << coner_count << endl;

			for (int j = 0; j < approx.size(); j++) {
				piece_x[frame_cnt][j] = approx[j].x;
				piece_y[frame_cnt][j] = approx[j].y;
			}
			frame_cnt++;
		}
	}
	Sort_Frame_IntersectionPoint(frame_dst, coner_count);
	Drawing_FramePoint();
	Get_FrameInformation(coner_count);
}



//**枠の角度と長さを取得**//
void Get_FrameInformation(int value) {
	for (int i = frame_count_angle; i < frame_cnt; i++) {
		for (int j = 0; j < value; j++) {
			//三点の座標から角度を求める
			if (j == value - 1) {
				BC_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[0];
				BA_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[j - 1];
				BC_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[0];
				BA_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[j - 1];
			}
			else if (j == 0) {
				BC_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[j + 1];
				BA_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[value - 1 - j];
				BC_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[j + 1];
				BA_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[value - 1 - j];
			}
			else {
				BC_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[j + 1];
				BA_x = frame_inf[i].Intersection_x[j] - frame_inf[i].Intersection_x[j - 1];
				BC_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[j + 1];
				BA_y = frame_inf[i].Intersection_y[j] - frame_inf[i].Intersection_y[j - 1];
			}
			frame_inf[i].piece_inf = value;
			//枠の長さを計算
			frame_inf[i].length[j] = sqrt((BC_x)* (BC_x)+(BC_y)* (BC_y));

			cout << "長さ=" << frame_inf[i].length[j] << endl;

			//ルートの計算
			double sq_1 = cvSqrt(BC_x*BC_x + BC_y*BC_y);
			double sq_2 = cvSqrt(BA_x*BA_x + BA_y*BA_y);

			double BCA = ((BC_x*BA_x) + (BC_y*BA_y));//分子
			double sq_12 = (sq_1*sq_2);//分母

			//角度の計算
			double ang = BCA / sq_12;
			double angle = acos(ang);

			angle = angle * 180 / PI;
			num = num + angle;

			frame_inf[i].angle[j] = angle;

			cout << "角度=" << frame_inf[i].angle[j] << endl;
		}
		cout << "合計の角度=" << num << endl;

		int flag_1 = 1;
		int all_angle = num;

		if (all_angle != (value - 2) * 180) {
			flag_1 = 0;
		}

		if (flag_1 == 0) {
			Convert_FrameAngle(0, value, n - 1, n - 1, i);
			all_angle = num;
			if (all_angle != (value - 2) * 180) {
				n = 2;
				Convert_FrameAngle(0, value, n - 1, n - 1, i);
				all_angle = num;
				if (all_angle != (value - 2) * 180) {
					n = 3;
					Convert_FrameAngle(0, value, n - 1, n - 1, i);
					all_angle = num;
					if (all_angle != (value - 2) * 180) {
						n = 4;
						Convert_FrameAngle(0, value, n - 1, n - 1, i);
						all_angle = num;
						if (all_angle != (value - 2) * 180) {
							Convert_Cmd_FrameAngel(value, i);
						}
					}
				}
			}
		}
		cout << endl;
		frame_count_angle++;
		cout << "合計の角度=" << num << endl;
		n = 1;
		num = 0;
	}
}



//**配置後の枠の情報抽出**//
/*
void Get_AfterPlacement_Information(Mat value) {
Mat LabelImg;
Mat stats;
Mat centroids;
Mat bin;
int placement_count = 0;

//配置後の画像を保存
Mat Save_Frame = value.clone();

//画像をぼかす（隙間を認識しないため）
GaussianBlur(Save_Frame, bin, cv::Size(11, 11), 10, 10);

cvtColor(bin, bin, CV_BGR2GRAY);
//二値化
threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
bin = ~bin;

int nLab = connectedComponentsWithStats(bin, LabelImg, stats, centroids);

cout << nLab;
//ROIの設定
for (int i = 2; i < nLab; ++i) {
int *param = stats.ptr<int>(i);

int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];



if (area > 100 && placement_count == 0) {//邪魔な領域除去処理
result_img[frame_cnt] = Mat(Save_Frame, Rect(x + 5, y + 5, width - 10, height - 10));
frame_dst[frame_cnt] = Mat(bin, Rect(x + 5, y + 5, width - 10, height - 10));


//輪郭の座標リスト
std::vector< std::vector< cv::Point > > contours;

//輪郭取得
cv::findContours(frame_dst[frame_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
std::vector< cv::Point > approx;
for (auto contour = contours.begin(); contour != contours.end(); contour++) {
//輪郭を直線近似する
approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
}

coner_count = approx.size();
cout << "輪郭は=" << coner_count << endl;
for (int j = 0; j < approx.size(); j++) {
piece_x[frame_cnt][j] = approx[j].x;
piece_y[frame_cnt][j] = approx[j].y;
}
frame_cnt++;
placement_count++;
}
}
Sort_Frame_IntersectionPoint(frame_dst, coner_count);
Drawing_FramePoint();
Get_FrameInformation(coner_count);
}
*/



//**輪郭追跡を行い交点の座標を時計回りにソート**//
void Sort_Piece_IntersectionPoint(int value) {
	int sx, sy;
	int px, py;
	int vec;       //調査点フラグ
	int IsFirst;   //
	int bEnd = false;
	double length;

	for (int i = count_chase; i < piece_cnt; ++i) {

		bEnd = false;

		//画像内を捜査し有効画素を探す
		for (sy = 0; sy < bin_img[i].rows; sy++)
		{
			for (sx = 0; sx < bin_img[i].cols; sx++)
			{
				if (bin_img[i].at<unsigned char>(sy, sx) != 0) {
					bEnd = true;
				}
				if (bEnd) { break; }
			}
			if (bEnd) { break; }
		}


		if (sx < bin_img[i].cols) {
			px = sx;
			py = sy;
			vec = 2;
			IsFirst = 1;

			//追跡開始点と追跡点が同じ座標なるまで輪郭追跡処理
			while (px != sx || py != sy || IsFirst == 1) {
				switch (vec) {
				case 0:    //右上を調査
					if (bin_img[i].at<unsigned char>(py - 1, px + 1) != 0) {
						px = px + 1; py = py - 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 6;
						break;
					}
				case 1:    //右を調査
					if (bin_img[i].at<unsigned char>(py, px + 1) != 0) {
						px = px + 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 0;
						break;
					}
				case 2:    //右下を調査
					if (bin_img[i].at<unsigned char>(py + 1, px + 1) != 0) {
						px = px + 1; py = py + 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 0;
						break;
					}
				case 3:    //下を調査
					if (bin_img[i].at<unsigned char>(py + 1, px) != 0) {
						py = py + 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 2;
						break;
					}

				case 4:    //左下を調査
					if (bin_img[i].at<unsigned char>(py + 1, px - 1) != 0) {
						px = px - 1; py = py + 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 2;
						break;
					}

				case 5:    //左を調査
					if (bin_img[i].at<unsigned char>(py, px - 1) != 0) {
						px = px - 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 4;
						break;
					}
					else {
						//孤立点であった場合
						if (IsFirst == 1) {
							IsFirst = 0;
							break;
						}
					}

				case 6:    //左上を調査
					if (bin_img[i].at<unsigned char>(py - 1, px - 1) != 0) {
						px = px - 1; py = py - 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 4;
						break;
					}
				case 7:    //上を調査
					if (bin_img[i].at<unsigned char>(py - 1, px) != 0) {
						px = px; py = py - 1;
						for (int j = 0; j<value; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length< 6) {
								piece_non[i].Intersection_x[counter] = piece_x[i][j];
								piece_non[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 6;
						break;
					}
					vec = 0;
				}
			}
		}
		count_chase++;
		counter = 0;
	}
}



//**輪郭追跡を行い交点の座標を時計回りにソート**//
void Sort_Frame_IntersectionPoint(Mat value[DATABASE], int val) {
	int sx, sy;
	int px, py;
	int vec;       //調査点フラグ
	int IsFirst;   //
	int bEnd = false;
	double length;

	for (int i = frame_count_chase; i < frame_cnt; ++i) {

		//画像内を捜査し有効画素を探す
		for (sy = 0; sy < value[i].rows; sy++)
		{
			for (sx = 0; sx < value[i].cols; sx++)
			{
				if (value[i].at<unsigned char>(sy, sx) != 0) {
					bEnd = true;
				}
				if (bEnd) { break; }
			}
			if (bEnd) { break; }
		}

		if (sx < value[i].cols) {
			px = sx;
			py = sy;
			vec = 2;
			IsFirst = 1;

			//追跡開始点と追跡点が同じ座標なるまで輪郭追跡処理
			while (px != sx || py != sy || IsFirst == 1) {
				switch (vec) {
				case 0:    //右上を調査
					if (value[i].at<unsigned char>(py - 1, px + 1) != 0) {
						px = px + 1; py = py - 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 6;
						break;
					}
				case 1:    //右を調査
					if (value[i].at<unsigned char>(py, px + 1) != 0) {
						px = px + 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 0;
						break;
					}
				case 2:    //右下を調査
					if (value[i].at<unsigned char>(py + 1, px + 1) != 0) {
						px = px + 1; py = py + 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 0;
						break;
					}
				case 3:    //下を調査
					if (value[i].at<unsigned char>(py + 1, px) != 0) {
						py = py + 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 2;
						break;
					}

				case 4:    //左下を調査
					if (value[i].at<unsigned char>(py + 1, px - 1) != 0) {
						px = px - 1; py = py + 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						IsFirst = 0;
						vec = 2;
						break;
					}

				case 5:    //左を調査
					if (value[i].at<unsigned char>(py, px - 1) != 0) {
						px = px - 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 4;
						break;
					}
					else {
						//孤立点であった場合
						if (IsFirst == 1) {
							IsFirst = 0;
							break;
						}
					}

				case 6:    //左上を調査
					if (value[i].at<unsigned char>(py - 1, px - 1) != 0) {
						px = px - 1; py = py - 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 4;
						break;
					}
				case 7:    //上を調査
					if (value[i].at<unsigned char>(py - 1, px) != 0) {
						px = px; py = py - 1;
						for (int j = 0; j < val; j++) {
							BC_x = (piece_x[i][j] - px);
							BC_y = (piece_y[i][j] - py);
							length = cvSqrt((BC_x*BC_x) + (BC_y*BC_y));
							if (-6 < length && length < 6) {
								frame_inf[i].Intersection_x[counter] = piece_x[i][j];
								frame_inf[i].Intersection_y[counter] = piece_y[i][j];
								piece_x[i][j] = 0;
								piece_y[i][j] = 0;
								counter = counter + 1;
							}
						}
						vec = 6;
						break;
					}
					vec = 0;
				}
			}
			frame_count_chase++;
			counter = 0;
		}
	}
}



//**外角の角度を内閣に変換する（ピース）**//
void Convert_PieceAngle(int start, int end, int depth, int maxdepth, int number) {
	int j, nPrintCnt;
	int nowdep = maxdepth - depth;
	double num1 = n * 360;
	double num4 = 0;
	double all_angle1 = num;
	for (j = start; j < end; j++) {
		p[nowdep] = j;
		if (depth>0) {
			Convert_PieceAngle(j + 1, end, depth - 1, maxdepth, number);
		}
		else {
			for (nPrintCnt = 0; nPrintCnt < nowdep + 1; nPrintCnt++) {
				all_angle1 -= piece_non[number].piece_angle[p[nPrintCnt]];
				num1 -= piece_non[number].piece_angle[p[nPrintCnt]];
			}
			num4 = all_angle1 + num1;
			if ((end - 2) * 180 <= num4 && num4<(end - 2) * 180 + 0.1) {
				for (nPrintCnt = 0; nPrintCnt < nowdep + 1; nPrintCnt++) {
					piece_non[number].piece_angle[p[nPrintCnt]] = 360 - piece_non[number].piece_angle[p[nPrintCnt]];
				}
				num = num4;
			}
		}
		all_angle1 = num;
		num1 = n * 360;
	}
}



//**手動偽角度除去処理**//
void Convert_Cmd_PieceAngel(int val, int val1) {
	//value=角形データ
	int false_angle[DATABASE];
	int count_true = 0;
	int flag_1 = 0;
	imshow("1", dst[val1]);
	waitKey(0);
	destroyAllWindows();
	while (flag_1 == 0) {
		cout << "どの角度ですか？(終了＝マイナスの値)" << endl;
		cin >> false_angle[count_true];
		if (false_angle[count_true] > 0) {
			count_true++;
		}
		else {
			flag_1 = 1;
		}
	}
	for (int j = 0; j < count_true; j++) {
		if (false_angle[j] > 0) {
			num = num - piece_non[val1].piece_angle[false_angle[j]];
			piece_non[val1].piece_angle[false_angle[j]] = 360 - piece_non[val1].piece_angle[false_angle[j]];
			num = num + piece_non[val1].piece_angle[false_angle[j]];
		}
	}
	cout << "訂正結果↓" << endl << num << endl << endl;
}



//**外角の角度を内角に変換する（枠）**//
void Convert_FrameAngle(int start, int end, int depth, int maxdepth, int number) {
	int j, nPrintCnt;
	int nowdep = maxdepth - depth;
	double num1 = n * 360;
	double num4 = 0;
	double all_angle1 = num;
	for (j = start; j < end; j++) {
		p[nowdep] = j;
		if (depth>0) {
			Convert_FrameAngle(j + 1, end, depth - 1, maxdepth, number);
		}
		else {
			for (nPrintCnt = 0; nPrintCnt < nowdep + 1; nPrintCnt++) {
				all_angle1 -= frame_inf[number].angle[p[nPrintCnt]];
				num1 -= frame_inf[number].angle[p[nPrintCnt]];
			}
			num4 = all_angle1 + num1;
			if ((end - 2) * 180 <= num4 && num4<(end - 2) * 180 + 0.1) {
				for (nPrintCnt = 0; nPrintCnt < nowdep + 1; nPrintCnt++) {
					frame_inf[number].angle[p[nPrintCnt]] = 360 - frame_inf[number].angle[p[nPrintCnt]];
				}
				num = num4;
			}
		}
		all_angle1 = num;
		num1 = n * 360;
	}
}



//**手動偽角度除去処理**//
void Convert_Cmd_FrameAngel(int val, int val1) {
	int false_angle[DATABASE]; //value=角形データ
	int count_true = 0;
	int flag_1 = 0;
	imshow("1", result_img[val1]);
	waitKey(0);
	destroyAllWindows();
	while (flag_1 == 0) {
		cout << "どの角度ですか？(終了＝マイナスの値)" << endl;
		cin >> false_angle[count_true];
		if (false_angle[count_true] > 0) {
			count_true++;
		}
		else {
			flag_1 = 1;
		}
	}
	for (int j = 0; j < count_true; j++) {
		if (false_angle[j] > 0) {
			num = num - piece_non[val1].piece_angle[false_angle[j]];
			frame_inf[val1].angle[false_angle[j]] = 360 - frame_inf[val1].angle[false_angle[j]];
			num = num + frame_inf[val1].angle[false_angle[j]];
		}
	}
	cout << "訂正結果↓" << endl << num << endl << endl;
}



//**交点を表示**//
void Drawing_FramePoint() {
	int d = 1;

	for (int i = frame_count_print; i < frame_cnt; i++) {
		for (int j = 0; j < coner_count; j++) {
			stringstream h;
			h << d;
			putText(result_img[i], h.str(), Point(frame_inf[i].Intersection_x[j], frame_inf[i].Intersection_y[j]), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 255, 255), 2);
			circle(result_img[i], Point(frame_inf[i].Intersection_x[j], frame_inf[i].Intersection_y[j]), 1, Scalar(0, 0, 255), -1);
			circle(result_img[i], Point(frame_inf[i].Intersection_x[j], frame_inf[i].Intersection_y[j]), 10, Scalar(0, 0, 255), 1);
			d++;
			cout << Point(frame_inf[i].Intersection_x[j], frame_inf[i].Intersection_y[j]) << endl;
		}
		frame_count_print++;
		d = 0;
	}
}



//**交点を表示**//
void Drawing_PiecePoint(int val1) {
	int d = 0;

	for (int i = count_print; i < piece_cnt; i++) {
		for (int j = 0; j < val1; j++) {
			stringstream h;
			h << d;
			putText(Piece_ROI[i], h.str(), Point(piece_non[i].Intersection_x[j], piece_non[i].Intersection_y[j]), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 255, 255), 2);
			circle(Piece_ROI[i], Point(piece_non[i].Intersection_x[j], piece_non[i].Intersection_y[j]), 1, Scalar(0, 0, 255), -1);
			circle(Piece_ROI[i], Point(piece_non[i].Intersection_x[j], piece_non[i].Intersection_y[j]), 10, Scalar(0, 0, 255), 1);
			d++;
		}
		d = 0;
		count_print++;
	}
}



//**認識したピースをすべて表示**//
void Display_Image() {
	for (int i = 0; i < Auto_img_count; i++) {
		imshow("1111111111" + i, sample[i]);
	}
	waitKey(0);
	destroyAllWindows();
}



//**線分交差判定(当たり判定)**//
/*
int Judge_Intersection(int p1x, int p2x, int p3x, int p4x,
int p1y, int p2y, int p3y, int p4y) {
if (((p1x - p2x) * (p3y - p1y) + (p1y - p2y) * (p1x - p3x))
* ((p1x - p2x) * (p4y - p1y) + (p1y - p2y) * (p1x - p4x)) < 0) {

if (((p3x - p4x) * (p1y - p3y) + (p3y - p4y) * (p3x - p1x))
* ((p3x - p4x) * (p2y - p3y) + (p3y - p4y) * (p3x - p2x)) < 0) {
//交差したら１を返す。
return(1);
}
}
//交差していなかったら0を返す。
else return(0);
}
*/



//**ピースの重複判定**//
/*
int Hit_PieceTest(int val) {
//並べられている個数繰り返す
for (int i = val; i > 0; i--) {
//基準となるピース
for (int j = 0; j < piece_non[cand_sort[val] / 100].piece_inf; j++) {
//今まで並べられたピース
for (int h = 0; h < piece_non[cand_sort[i - 1] / 100].piece_inf; h++) {
//jとhが最大の値だったら
if (j == piece_non[cand_sort[val] / 100].piece_inf - 1 && h == piece_non[cand_sort[i - 1] / 100].piece_inf - 1) {
//線分が交差していたら1を交差していなければ0を返す
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][0], piece_coo_x[i - 1][h], piece_coo_x[i - 1][0], piece_coo_y[val][j], piece_coo_y[val][0], piece_coo_y[i - 1][h], piece_coo_y[i - 1][0]) == 1) {
return 1;
}
}
//hが最大の値だったら
else if (h == piece_non[cand_sort[i - 1] / 100].piece_inf - 1) {
//線分が交差していたら1を交差していなければ0を返す
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][j + 1], piece_coo_x[i - 1][h], piece_coo_x[i - 1][0], piece_coo_y[val][j], piece_coo_y[val][j + 1], piece_coo_y[i - 1][h], piece_coo_y[i - 1][0]) == 1) {
return 1;
}
}
//jが最大の値だったら
else if (j == piece_non[cand_sort[val] / 100].piece_inf - 1) {
//線分が交差していたら1を交差していなければ0を返す
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][0], piece_coo_x[i - 1][h], piece_coo_x[i - 1][h + 1], piece_coo_y[val][j], piece_coo_y[val][0], piece_coo_y[i - 1][h], piece_coo_y[i - 1][h + 1]) == 1) {
return 1;
}
}
//それ以外
else {
//線分が交差していたら1を交差していなければ0を返す
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][j + 1], piece_coo_x[i - 1][h], piece_coo_x[i - 1][h + 1], piece_coo_y[val][j], piece_coo_y[val][j + 1], piece_coo_y[i - 1][h], piece_coo_y[i - 1][h + 1]) == 1) {
return 1;
}
}
}
}
}
return 0;
}
*/






