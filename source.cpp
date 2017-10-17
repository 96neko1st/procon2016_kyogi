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


//**		�s�[�X�̏����i�[����z��		**//
typedef struct piece {
	int piece_inf;					//���p�`���̏��
	int Intersection_x[32];			//�s�[�X�̌�_x���i�[
	int Intersection_y[32];			//�s�[�X�̌�_y���i�[
	double piece_angle[32];			//�s�[�X�̊p�x���i�[
	double piece_length[32];		//�s�[�X�̒������i�[
}piece;

piece piece_non[DATABASE];


//**		�g�̏����i�[����z��		**//
typedef struct FRAME {
	int piece_inf;					//���p�`���̏��
	int Intersection_x[DATABASE];	//�g�̌�_x���i�[
	int Intersection_y[DATABASE];	//�g�̌�_y���i�[
	double angle[DATABASE];			//�g�̊p�x���i�[
	double length[DATABASE];		//�g�̒������i�[
}frame;

frame frame_inf[DATABASE];


//�ċA�����Ɏg�p
int *p;


//���������ċA����(���[�v�̐[��)
int n = 1;


//�摜�J�E���g
int Auto_img_count = 0;


int Manual_img_count = 0;


//��_�̐���ۑ�
int coner_count = 3;


//�g�p����摜�̔ԍ�
int img_number;


//�s�[�X�̑S�̉摜�����T�C�Y���邽�߂�flag
int change_flag = 0;


//�g�p����R�}���h�̒l
int command = 0;


//x��_�ۑ�(�\�[�g�����O)
int piece_x[DATABASE][DATABASE];


//y��_�ۑ�(�\�[�g�����O)
int piece_y[DATABASE][DATABASE];


//�蓮�ŔF������摜�ۑ�
Mat src[32];


//�����ŔF������摜�ۑ�
Mat sample[32];


//�g�A���O�摜�ۑ�
Mat src_img[2];


//�g�A����摜�ۑ�
Mat dst_img;


//�g�̌��摜��ۑ�
Mat frame_src;


//ROI��1�̘g�̗̈�𒊏o(���摜)
Mat result_img[DATABASE];


//ROI��1�̘g�̗̈�𒊏o(�����摜)
Mat frame_dst[DATABASE];


//ROI��1�̃s�[�X�𒊏o(���摜)
Mat Piece_ROI[DATABASE];


//ROI��1�̃s�[�X�𒊏o(�F�t��)
Mat dst[DATABASE];


//ROI��1�̃s�[�X�𒊏o(�����摜)
Mat bin_img[DATABASE];


//�O�_�̍��WABC��BC��x���W�̍���
double BC_x = 0;


//�O�_�̍��WABC��BA��x���W�̍���
double BA_x = 0;


//�O�_�̍��WABC��BC��y���W�̍���
double BC_y = 0;


//�O�_�̍��WABC��BA��y���W�̍���
double BA_y = 0;


//�p�x�̍��v
double num = 0;


//�s�[�X�̌��J�E���g
int piece_cnt = 0;


//�g�̌��J�E���g
int frame_cnt = 0;


//��_�̐��J�E���g
int counter = 0;


//�֊s�ǐՎ��s�񐔋L���J�E���g(�g)
int frame_count_chase = 0;


//�p�x�擾���s�񐔃J�E���g(�g)
int frame_count_angle = 0;


//�g�\�����s�񐔃J�E���g
int frame_count_print = 0;


//��_�����`��J�E���g
int Auto_count = 0;


//�֊s�ǐՎ��s�񐔋L���J�E���g(�s�[�X)
int count_chase = 0;


//�p�x�擾���s�񐔃J�E���g(�s�[�X)
int count_angle = 0;


//�s�[�X�\�����s�񐔃J�E���g
int count_print = 0;


//**���͂����֊s���擾**//
void Get_ManualPiece(Mat value[DATABASE]);


//**�擾�����摜�̃s�[�X�̗֊s���擾**//
void Get_AutoPiece(Mat value[DATABASE]);


//**�s�[�X�̊p�x�Ⓑ�����擾**//
void Get_PieceInformation(int val);


//**�摜��A�����Č�_�Ɨ֊s���擾**//
void Get_Frame_IntersectionPoint();


//**��_����g�̊p�x�ƒ������擾**//
void Get_FrameInformation(int value);


//**�s�[�X�z�u��̏����擾**//
void Get_AfterPlacement_Information(Mat value);


//**�֊s�ǐՎ��v���\�[�g**//
void Sort_Piece_IntersectionPoint(int value);


//**�֊s��ǐՂ���_�����v���Ƀ\�[�g**//
void Sort_Frame_IntersectionPoint(Mat value[DATABASE], int val);


//**�s�[�X�̊O�p�̊p�x����p�ɕϊ�����i����)**//
void Convert_PieceAngle(int start, int end, int depth, int maxdepth, int number);


//**�s�[�X�̊O�p�̊p�x����p�ɕϊ�����i�蓮)**//
void Convert_Cmd_PieceAngel(int val, int val1);


//**�g�̊O�p�̊p�x����t�ɕϊ�����(�蓮)**//
void Convert_Cmd_FrameAngel(int val, int val1);


//**�g�̊O�p�̊p�x����p�ɕϊ�����i�����j**//
void Convert_FrameAngle(int start, int end, int depth, int maxdepth, int number);


//**�g�̌�_���摜�ɕ`��**//
void Drawing_FramePoint();


//**�s�[�X�̌�_���摜�ɕ`��**//
void Drawing_PiecePoint(int val1);


//**�s�[�X�̉摜��\��**//
void Display_Image();


//**������������**//
//int Judge_Intersection(int p1x, int p2x, int p3x, int p4x, int p1y, int p2y, int p3y, int p4y);


//**�s�[�X�̓����蔻��**//
//int Hit_PieceTest(int val);



int main()
{

	p = (int*)malloc(sizeof(int) * 16);

	//�s�[�X�̊p���Ƃ̉摜
	src[6] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src003.jpg");
	//�������œǂݎ��摜
	sample[0] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src002.jpg");
	sample[1] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\src004.jpg");

	src_img[0] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\c002.jpg");
	src_img[1] = imread("C:\\opencv-3.1.0\\sources\\samples\\data\\c001.jpg");

	int flag = 0;

	while (flag == 0) {
		cout << "0:�I�� 1:�蓮���� 2:�������� 3:�g 4:�ׂ荇�����s�[�X������ 5:�t������ 6:�摜�\��" << endl;
		cin >> command;
		switch (command) {
		case 0:	//�I��
			if (frame_cnt == 0) { cout << "�g�̏������Ă�������" << endl;  break; }
			flag = 1;
			break;
		case 1:	//���̓��[�h
			cout << "���p�`�ł����H" << endl;
			cin >> img_number;
			if (src[img_number].empty()) { break; }
			Get_ManualPiece(src);
			Manual_img_count++;
			break;
		case 2:	//�������̓��[�h
			cout << "���͉摜��I�����Ă�������" << endl;
			cin >> img_number;
			if (sample[img_number].empty()) { break; }
			Get_AutoPiece(sample);
			Auto_img_count++;
			break;
		case 3:	//�g�̏�񒊏o���[�h
			if (src_img[0].empty() || src_img[1].empty()) { break; }
			Get_Frame_IntersectionPoint();
			imshow("�g", result_img[frame_cnt - 1]);
			waitKey();
			break;
		case 4:	//piece�摜�\��
			Display_Image();
			break;
		default:
			cout << "�I�����ɂȂ��ԍ��ł�" << endl;
			break;
		}
	}

	cout << piece_cnt;
	waitKey();
	return 0;
}



//**ROI�̗̈���擾����_�̍��W���擾**//
void Get_AutoPiece(Mat value[DATABASE]) {
	//�ǂݍ��݉摜�����T�C�Y����
	resize(value[img_number], value[img_number], cv::Size(), a, a);

	//��l���摜�ۑ�
	Mat bin;
	cvtColor(value[img_number], bin, CV_BGR2GRAY);
	// ��l��
	threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	//���׃����O����
	Mat LabelImg;
	Mat stats;
	Mat centroids;

	int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids);


	// ���x�����O���ʂ̕`��F������
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}


	// ���x�����O���ʂ̕`��
	Mat Dst(value[img_number].size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROI�̐ݒ�
	for (int i = 1; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int Area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		//�ʐς�100�ȉ��̗̈�͏���
		if (Area > 100) {
			//ROI��1�̃s�[�X�𒊏o(���摜)
			Piece_ROI[piece_cnt] = Mat(value[img_number], Rect(x - 5, y - 5, width + 10, height + 10));

			//ROI��1�̃s�[�X���o(�F�t��)
			dst[piece_cnt] = Mat(Dst, Rect(x - 5, y - 5, width + 10, height + 10));


			//ROI��1�̃s�[�X���o(�����摜)
			bin_img[piece_cnt] = Mat(bin, Rect(x - 5, y - 5, width + 10, height + 10));

			//�֊s�̍��W���X�g
			std::vector< std::vector< cv::Point > > contours;

			//�֊s�擾
			findContours(bin_img[piece_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			std::vector< cv::Point > approx;

			for (auto contour = contours.begin(); contour != contours.end(); contour++) {
				//���o�������W��`��
				polylines(dst[piece_cnt], *contour, true, cv::Scalar(0, 255, 255), 1);

				//�֊s�𒼐��ߎ�����
				approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
			}


			//�d�S�Ƀs�[�X�ԍ���\��
			double *param = centroids.ptr<double>(i);
			int x_point = static_cast<int>(param[0]);
			int y_point = static_cast<int>(param[1]);

			stringstream b;
			b << piece_cnt;
			putText(sample[img_number], b.str(), Point(x_point, y_point), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 255), 2);



			//�s�[�X�̗֊s�����i�[
			coner_count = approx.size();
			cout << "�֊s��=" << coner_count << endl;


			//�s�[�X�̌�_���W���i�[
			for (int j = 0; j < approx.size(); j++) {
				piece_x[piece_cnt][j] = approx[j].x;
				piece_y[piece_cnt][j] = approx[j].y;
			}


			//�s�[�X�J�E���g�𑝂₷
			piece_cnt = piece_cnt + 1;


			Sort_Piece_IntersectionPoint(coner_count);
			Drawing_PiecePoint(coner_count);
			Get_PieceInformation(coner_count);
		}
	}
}



//**���͂����֊s�̌�_�̍��W���擾**//
void Get_ManualPiece(Mat value[DATABASE]) {
	Mat bin;
	resize(value[img_number], value[img_number], cv::Size(), a, a);
	//�O���C�X�P�[����
	cvtColor(value[img_number], bin, CV_BGR2GRAY);
	// ��l��
	threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//���׃����O����
	Mat LabelImg;
	Mat stats;
	Mat centroids;
	int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids); //�F��������

	// ���x�����O���ʂ̕`��F������
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	// ���x�����O���ʂ̕`��
	Mat Dst(value[img_number].size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROI�̐ݒ�
	for (int i = 2; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		if (area > 100) {//�ז��ȗ̈揜������
			Piece_ROI[piece_cnt] = Mat(value[img_number], Rect(x - 5, y - 5, width + 10, height + 10));
			dst[piece_cnt] = Mat(Dst, Rect(x - 5, y - 5, width + 10, height + 10));

			bin_img[piece_cnt] = Mat(bin, Rect(x - 5, y - 5, width + 10, height + 10));

			//harris�R�[�i�\���o
			vector<Point2f> coners;
			goodFeaturesToTrack(bin_img[piece_cnt], coners, img_number, 0.1, 3, Mat(), 3, true);
			vector<Point2f>::iterator it_coners = coners.begin();
			for (; it_coners != coners.end(); ++it_coners) {
				//�z��Ɍ�_���W���i�[
				piece_x[piece_cnt][counter] = it_coners->x;
				piece_y[piece_cnt][counter] = it_coners->y;
				counter += 1;
			}


			//�d�S�Ƀs�[�X�ԍ���\��
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



//**�s�[�X�̊p�x�ƒ��������߂�**//
void Get_PieceInformation(int val) {
	for (int i = count_angle; i < piece_cnt; ++i) {
		cout << i << "�Ԗ�" << endl;
		for (int j = 0; j <val; j++) {
			//�O�_�̍��W����p�x�����߂�
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

			//�s�[�X�̊p�`�̏��}��
			piece_non[i].piece_inf = val;
			//�s�[�X�̒������v�Z
			piece_non[i].piece_length[j] = sqrt((BC_x)* (BC_x)+(BC_y)* (BC_y));

			cout << "����=" << piece_non[i].piece_length[j] << endl;

			//���[�g�̌v�Z
			double sq_1 = cvSqrt(BC_x*BC_x + BC_y*BC_y);
			double sq_2 = cvSqrt(BA_x*BA_x + BA_y*BA_y);

			double BCA = ((BC_x*BA_x) + (BC_y*BA_y));//���q
			double sq_12 = (sq_1*sq_2);//����
			//�p�x�̌v�Z
			double ang = BCA / sq_12;
			double angle = acos(ang);

			angle = angle * 180 / PI;
			piece_non[i].piece_angle[j] = angle;
			num = num + angle;
			cout << "�p�x=" << piece_non[i].piece_angle[j] << endl;
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

		cout << "���v�̊p�x=" << num << endl;
		n = 1;
		num = 0;
		cout << endl;
		count_angle++;
	}
}



//**�摜�A��**//
void Get_Frame_IntersectionPoint() {
	resize(src_img[0], src_img[0], cv::Size(), a, a);
	resize(src_img[1], src_img[1], cv::Size(), a, a);

	//�摜���A��
	hconcat(src_img, 2, dst_img);
	Mat frame_src = dst_img.clone();
	//�O���C�X�P�[����
	cvtColor(dst_img, dst_img, CV_BGR2GRAY);
	//��l��
	threshold(dst_img, dst_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//���]
	dst_img = ~dst_img;

	//���x�����O���
	Mat LabelImg;
	Mat stats;
	Mat centroids;
	int nLab = cv::connectedComponentsWithStats(dst_img, LabelImg, stats, centroids);

	//ROI�̐ݒ�
	for (int i = 3; i < nLab; ++i) {
		int *param = stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

		if (area > 100) {//�ז��ȗ̈揜������
			//ROI�ŗ̈���擾(���摜)
			result_img[frame_cnt] = Mat(frame_src, Rect(x - 5, y - 5, width + 10, height + 10));

			//ROI�ŗ̈���擾(�����摜)
			frame_dst[frame_cnt] = Mat(dst_img, Rect(x - 5, y - 5, width + 10, height + 10));

			//�֊s�̍��W���X�g
			std::vector< std::vector< cv::Point > > contours;

			//�֊s�擾
			findContours(frame_dst[frame_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			std::vector< cv::Point > approx;

			for (auto contour = contours.begin(); contour != contours.end(); contour++) {
				//�֊s�𒼐��ߎ�����
				cv::approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
			}

			coner_count = approx.size();

			cout << "�֊s��=" << coner_count << endl;

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



//**�g�̊p�x�ƒ������擾**//
void Get_FrameInformation(int value) {
	for (int i = frame_count_angle; i < frame_cnt; i++) {
		for (int j = 0; j < value; j++) {
			//�O�_�̍��W����p�x�����߂�
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
			//�g�̒������v�Z
			frame_inf[i].length[j] = sqrt((BC_x)* (BC_x)+(BC_y)* (BC_y));

			cout << "����=" << frame_inf[i].length[j] << endl;

			//���[�g�̌v�Z
			double sq_1 = cvSqrt(BC_x*BC_x + BC_y*BC_y);
			double sq_2 = cvSqrt(BA_x*BA_x + BA_y*BA_y);

			double BCA = ((BC_x*BA_x) + (BC_y*BA_y));//���q
			double sq_12 = (sq_1*sq_2);//����

			//�p�x�̌v�Z
			double ang = BCA / sq_12;
			double angle = acos(ang);

			angle = angle * 180 / PI;
			num = num + angle;

			frame_inf[i].angle[j] = angle;

			cout << "�p�x=" << frame_inf[i].angle[j] << endl;
		}
		cout << "���v�̊p�x=" << num << endl;

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
		cout << "���v�̊p�x=" << num << endl;
		n = 1;
		num = 0;
	}
}



//**�z�u��̘g�̏�񒊏o**//
/*
void Get_AfterPlacement_Information(Mat value) {
Mat LabelImg;
Mat stats;
Mat centroids;
Mat bin;
int placement_count = 0;

//�z�u��̉摜��ۑ�
Mat Save_Frame = value.clone();

//�摜���ڂ����i���Ԃ�F�����Ȃ����߁j
GaussianBlur(Save_Frame, bin, cv::Size(11, 11), 10, 10);

cvtColor(bin, bin, CV_BGR2GRAY);
//��l��
threshold(bin, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
bin = ~bin;

int nLab = connectedComponentsWithStats(bin, LabelImg, stats, centroids);

cout << nLab;
//ROI�̐ݒ�
for (int i = 2; i < nLab; ++i) {
int *param = stats.ptr<int>(i);

int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];



if (area > 100 && placement_count == 0) {//�ז��ȗ̈揜������
result_img[frame_cnt] = Mat(Save_Frame, Rect(x + 5, y + 5, width - 10, height - 10));
frame_dst[frame_cnt] = Mat(bin, Rect(x + 5, y + 5, width - 10, height - 10));


//�֊s�̍��W���X�g
std::vector< std::vector< cv::Point > > contours;

//�֊s�擾
cv::findContours(frame_dst[frame_cnt], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
std::vector< cv::Point > approx;
for (auto contour = contours.begin(); contour != contours.end(); contour++) {
//�֊s�𒼐��ߎ�����
approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
}

coner_count = approx.size();
cout << "�֊s��=" << coner_count << endl;
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



//**�֊s�ǐՂ��s����_�̍��W�����v���Ƀ\�[�g**//
void Sort_Piece_IntersectionPoint(int value) {
	int sx, sy;
	int px, py;
	int vec;       //�����_�t���O
	int IsFirst;   //
	int bEnd = false;
	double length;

	for (int i = count_chase; i < piece_cnt; ++i) {

		bEnd = false;

		//�摜����{�����L����f��T��
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

			//�ǐՊJ�n�_�ƒǐՓ_���������W�Ȃ�܂ŗ֊s�ǐՏ���
			while (px != sx || py != sy || IsFirst == 1) {
				switch (vec) {
				case 0:    //�E��𒲍�
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
				case 1:    //�E�𒲍�
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
				case 2:    //�E���𒲍�
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
				case 3:    //���𒲍�
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

				case 4:    //�����𒲍�
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

				case 5:    //���𒲍�
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
						//�Ǘ��_�ł������ꍇ
						if (IsFirst == 1) {
							IsFirst = 0;
							break;
						}
					}

				case 6:    //����𒲍�
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
				case 7:    //��𒲍�
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



//**�֊s�ǐՂ��s����_�̍��W�����v���Ƀ\�[�g**//
void Sort_Frame_IntersectionPoint(Mat value[DATABASE], int val) {
	int sx, sy;
	int px, py;
	int vec;       //�����_�t���O
	int IsFirst;   //
	int bEnd = false;
	double length;

	for (int i = frame_count_chase; i < frame_cnt; ++i) {

		//�摜����{�����L����f��T��
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

			//�ǐՊJ�n�_�ƒǐՓ_���������W�Ȃ�܂ŗ֊s�ǐՏ���
			while (px != sx || py != sy || IsFirst == 1) {
				switch (vec) {
				case 0:    //�E��𒲍�
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
				case 1:    //�E�𒲍�
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
				case 2:    //�E���𒲍�
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
				case 3:    //���𒲍�
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

				case 4:    //�����𒲍�
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

				case 5:    //���𒲍�
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
						//�Ǘ��_�ł������ꍇ
						if (IsFirst == 1) {
							IsFirst = 0;
							break;
						}
					}

				case 6:    //����𒲍�
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
				case 7:    //��𒲍�
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



//**�O�p�̊p�x����t�ɕϊ�����i�s�[�X�j**//
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



//**�蓮�U�p�x��������**//
void Convert_Cmd_PieceAngel(int val, int val1) {
	//value=�p�`�f�[�^
	int false_angle[DATABASE];
	int count_true = 0;
	int flag_1 = 0;
	imshow("1", dst[val1]);
	waitKey(0);
	destroyAllWindows();
	while (flag_1 == 0) {
		cout << "�ǂ̊p�x�ł����H(�I�����}�C�i�X�̒l)" << endl;
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
	cout << "�������ʁ�" << endl << num << endl << endl;
}



//**�O�p�̊p�x����p�ɕϊ�����i�g�j**//
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



//**�蓮�U�p�x��������**//
void Convert_Cmd_FrameAngel(int val, int val1) {
	int false_angle[DATABASE]; //value=�p�`�f�[�^
	int count_true = 0;
	int flag_1 = 0;
	imshow("1", result_img[val1]);
	waitKey(0);
	destroyAllWindows();
	while (flag_1 == 0) {
		cout << "�ǂ̊p�x�ł����H(�I�����}�C�i�X�̒l)" << endl;
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
	cout << "�������ʁ�" << endl << num << endl << endl;
}



//**��_��\��**//
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



//**��_��\��**//
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



//**�F�������s�[�X�����ׂĕ\��**//
void Display_Image() {
	for (int i = 0; i < Auto_img_count; i++) {
		imshow("1111111111" + i, sample[i]);
	}
	waitKey(0);
	destroyAllWindows();
}



//**������������(�����蔻��)**//
/*
int Judge_Intersection(int p1x, int p2x, int p3x, int p4x,
int p1y, int p2y, int p3y, int p4y) {
if (((p1x - p2x) * (p3y - p1y) + (p1y - p2y) * (p1x - p3x))
* ((p1x - p2x) * (p4y - p1y) + (p1y - p2y) * (p1x - p4x)) < 0) {

if (((p3x - p4x) * (p1y - p3y) + (p3y - p4y) * (p3x - p1x))
* ((p3x - p4x) * (p2y - p3y) + (p3y - p4y) * (p3x - p2x)) < 0) {
//����������P��Ԃ��B
return(1);
}
}
//�������Ă��Ȃ�������0��Ԃ��B
else return(0);
}
*/



//**�s�[�X�̏d������**//
/*
int Hit_PieceTest(int val) {
//���ׂ��Ă�����J��Ԃ�
for (int i = val; i > 0; i--) {
//��ƂȂ�s�[�X
for (int j = 0; j < piece_non[cand_sort[val] / 100].piece_inf; j++) {
//���܂ŕ��ׂ�ꂽ�s�[�X
for (int h = 0; h < piece_non[cand_sort[i - 1] / 100].piece_inf; h++) {
//j��h���ő�̒l��������
if (j == piece_non[cand_sort[val] / 100].piece_inf - 1 && h == piece_non[cand_sort[i - 1] / 100].piece_inf - 1) {
//�������������Ă�����1���������Ă��Ȃ����0��Ԃ�
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][0], piece_coo_x[i - 1][h], piece_coo_x[i - 1][0], piece_coo_y[val][j], piece_coo_y[val][0], piece_coo_y[i - 1][h], piece_coo_y[i - 1][0]) == 1) {
return 1;
}
}
//h���ő�̒l��������
else if (h == piece_non[cand_sort[i - 1] / 100].piece_inf - 1) {
//�������������Ă�����1���������Ă��Ȃ����0��Ԃ�
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][j + 1], piece_coo_x[i - 1][h], piece_coo_x[i - 1][0], piece_coo_y[val][j], piece_coo_y[val][j + 1], piece_coo_y[i - 1][h], piece_coo_y[i - 1][0]) == 1) {
return 1;
}
}
//j���ő�̒l��������
else if (j == piece_non[cand_sort[val] / 100].piece_inf - 1) {
//�������������Ă�����1���������Ă��Ȃ����0��Ԃ�
if (Judge_Intersection(piece_coo_x[val][j], piece_coo_x[val][0], piece_coo_x[i - 1][h], piece_coo_x[i - 1][h + 1], piece_coo_y[val][j], piece_coo_y[val][0], piece_coo_y[i - 1][h], piece_coo_y[i - 1][h + 1]) == 1) {
return 1;
}
}
//����ȊO
else {
//�������������Ă�����1���������Ă��Ȃ����0��Ԃ�
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






