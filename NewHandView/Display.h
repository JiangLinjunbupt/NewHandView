#pragma once

#include <GL\glew.h>
#include <GL\freeglut.h>
#include "Viewer.h"
#include "SubdivisionSurfaces.h"
#include "CloudPoint.h"
#include "Projection.h"
#include "HandControl.h"

using namespace cv;

class HandControl;
void MixShowResult(cv::Mat input1, cv::Mat input2);
namespace DS
{
	int i = 0;
	struct {
		cv::Point3f eye;
		cv::Point3f center;
		cv::Point3f up;
	}cameraPos;

	void moveMeFlat(bool isHorizontal, int step) {

		cv::Point3f lookAt = cameraPos.center - cameraPos.eye;
		cv::Point3f direction;
		if (isHorizontal)
		{
			direction = lookAt.cross(cameraPos.up);
		}
		else
		{
			direction = lookAt;
		}
		cameraPos.eye += direction*step*0.08;
		cameraPos.center += direction*step*0.08;

		glutPostRedisplay();
	}
#pragma region  Keybroad_event(show mesh or not)

	void menu(int op) {

		switch (op) {
		case 'Q':
		case 'q':
			exit(0);
		}
	}

	/* executed when a regular key is pressed */
	void keyboardDown(unsigned char key, int x, int y) {

		switch (key) {
		case 'q':
			config.show_mesh = true;
			config.show_point = false;
			config.show_skeleton = false;
			break;
		case 'w':
			config.show_mesh = false;
			config.show_point = true;
			config.show_skeleton = true;
			break;

		case  27:   // ESC
			exit(0);
		}
	}

	/* executed when a regular key is released */
	void keyboardUp(unsigned char key, int x, int y) {

	}

	/* executed when a special key is pressed */
	void keyboardSpecialDown(int k, int x, int y) {
		switch (k) {
		case GLUT_KEY_LEFT:
			moveMeFlat(true, -1); break;
		case GLUT_KEY_RIGHT:
			moveMeFlat(true, 1); break;
		case GLUT_KEY_UP:
			moveMeFlat(false, 1); break;
		case GLUT_KEY_DOWN:
			moveMeFlat(false, -1); break;
		}
	}

	/* executed when a special key is released */
	void keyboardSpecialUp(int k, int x, int y) {}
#pragma endregion  Keybroad_event(show mesh or not)


	/* reshaped window */
	void reshape(int width, int height) {

		GLfloat fieldOfView = 90.0f;
		glViewport(0, 0, (GLsizei)width, (GLsizei)height);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(cameraPos.eye.x, cameraPos.eye.y, cameraPos.eye.z,
			cameraPos.center.x, cameraPos.center.y, cameraPos.center.z,
			cameraPos.up.x, cameraPos.up.y, cameraPos.up.z);
	}

	/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
	void mouseClick(int button, int state, int x, int y) {
		control.mouse_click = 1;
		control.x = x;
		control.y = y;
	}

	/* executed when the mouse moves to position ('x', 'y') */
	//void logo() {
	//	glRasterPos2i(100, 100);
	//	glColor3d(0.0, 0.0, 1.0);
	//	const unsigned char kurff0[] = "kurff";
	//	glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff0);
	//	glRasterPos2i(-100, 100);
	//	glColor3d(0.0, 1.0, 0.0);  //(red ,green ,blue)
	//	const unsigned char kurff1[] = "kurff";
	//	glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff1);
	//	glRasterPos2i(100, -100);
	//	glColor3d(1.0, 0.0, 0.0);
	//	const unsigned char kurff2[] = "kurff";
	//	glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff2);
	//	glRasterPos2i(-100, -100);
	//	glColor3d(1.0, 1.0, 0);
	//	const unsigned char kurff3[] = "kurff";
	//	glutBitmapString(GLUT_BITMAP_HELVETICA_18, kurff3);
	//}

	/* render the scene */
	void draw() {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		gluPerspective(180, 1.5, -1000, 1000);
		glLoadIdentity();
		gluLookAt(cameraPos.eye.x, cameraPos.eye.y, cameraPos.eye.z,
			cameraPos.center.x, cameraPos.center.y, cameraPos.center.z,
			cameraPos.up.x, cameraPos.up.y, cameraPos.up.z);


		/* render the scene here */
		// glColor3d(1.0,1.0,1.0);
		if (config.show_point) {
			glPointSize(2);
			glBegin(GL_POINTS);
			glColor3d(1.0, 0.0, 0.0);
			for (int i = 0; i < model->vertices_update_.rows(); i++) {

				glVertex3d(model->vertices_update_(i, 0), model->vertices_update_(i, 1), model->vertices_update_(i, 2));

				//cout<< model->vertices_update_(i,0)<< " " << model->vertices_update_(i,1) <<" "<< model->vertices_update_(i,2)<<endl;
			}
			glEnd();

		}

		if (config.show_mesh) {
			if (_data.indices == nullptr) return;
			if (_data.vertices == nullptr) return;
			//glColor3d(0.0, 0.0, 1.0);
			//glEnableClientState(GL_VERTEX_ARRAY);
			//glVertexPointer(3, GL_FLOAT, 0, _data.vertices);
			//glEnableClientState(GL_COLOR_ARRAY);
			//glColorPointer(3, GL_FLOAT, 0, _data.colors);
			////glDrawElements(GL_TRIANGLE_STRIP, 12, GL_UNSIGNED_BYTE, indices);
			//glDrawElements(GL_TRIANGLES, 3 * _data.num_face, GL_UNSIGNED_INT, _data.indices);

			//// deactivate vertex arrays after drawing
			//glDisableClientState(GL_VERTEX_ARRAY);

			for (int i = 0; i < SS::disPatches.size(); i++)
			{
				//画出三角网格
				//glLineWidth(1);
				//glColor3f(0.0, 1.0, 0);
				//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				//glFrontFace(GL_CCW);              // 设置逆时针方向为正面 
				//glBegin(GL_TRIANGLES);
				//glVertex3f(model->vertices_update_(model->faces_(i, 0), 0), model->vertices_update_(model->faces_(i, 0), 1), model->vertices_update_(model->faces_(i, 0), 2));
				//glVertex3f(model->vertices_update_(model->faces_(i, 1), 0), model->vertices_update_(model->faces_(i, 1), 1), model->vertices_update_(model->faces_(i, 1), 2));
				//glVertex3f(model->vertices_update_(model->faces_(i, 2), 0), model->vertices_update_(model->faces_(i, 2), 1), model->vertices_update_(model->faces_(i, 2), 2));
				//glEnd();


				//画出三角网格
				glLineWidth(1);
				glColor3f(0.0, 1.0, 0);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glFrontFace(GL_CCW);              // 设置逆时针方向为正面 
				glBegin(GL_TRIANGLES);
				glVertex3f(SS::disVertices[SS::disPatches[i].v_idx(0)].position.x, SS::disVertices[SS::disPatches[i].v_idx(0)].position.y, SS::disVertices[SS::disPatches[i].v_idx(0)].position.z);
				glVertex3f(SS::disVertices[SS::disPatches[i].v_idx(1)].position.x, SS::disVertices[SS::disPatches[i].v_idx(1)].position.y, SS::disVertices[SS::disPatches[i].v_idx(1)].position.z);
				glVertex3f(SS::disVertices[SS::disPatches[i].v_idx(2)].position.x, SS::disVertices[SS::disPatches[i].v_idx(2)].position.y, SS::disVertices[SS::disPatches[i].v_idx(2)].position.z);
				glEnd();


				//画出所有法向量
				//double Fx = (model->vertices_update_(model->faces_(i, 0), 0) + model->vertices_update_(model->faces_(i, 2), 0)) / 4 + model->vertices_update_(model->faces_(i, 1), 0)/2;
				//double Fy = (model->vertices_update_(model->faces_(i, 0), 1) + model->vertices_update_(model->faces_(i, 2), 1)) / 4 + model->vertices_update_(model->faces_(i, 1), 1)/2;
				//double Fz = (model->vertices_update_(model->faces_(i, 0), 2) + model->vertices_update_(model->faces_(i, 2), 2)) / 4 + model->vertices_update_(model->faces_(i, 1), 2)/2;

				//glLineWidth(1);
				//glColor3f(0.0, 1.0, 0);
				//glBegin(GL_LINES);
				//glVertex3f(Fx, Fy, Fz);
				////cout << "x: " << model->faces_normals(i, 0) << "  y: " << model->faces_normals(i, 1) << "  z: " << model->faces_normals(i, 2) << endl;
				//glVertex3f(Fx + 5*model->faces_normals(i,0), Fy + 5*model->faces_normals(i, 1), Fz + 5*model->faces_normals(i, 2));
				//glEnd();
			}


			//画出可见部分面部分的法向量
			//for (int i = 0; i < model->visible_faces.size(); i++)
			//{

			// //这里注意有手指甲的存在，因此，在后续的点云到模型的距离的手，需要加重对于手指尖端部分顶点的位置在点云位置前面的惩罚！！
			// double Fx = (model->vertices_update_(model->faces_(model->visible_faces[i], 0), 0) + model->vertices_update_(model->faces_(model->visible_faces[i], 2), 0)) / 4 + model->vertices_update_(model->faces_(model->visible_faces[i], 1), 0)/2;
			// double Fy = (model->vertices_update_(model->faces_(model->visible_faces[i], 0), 1) + model->vertices_update_(model->faces_(model->visible_faces[i], 2), 1)) / 4 + model->vertices_update_(model->faces_(model->visible_faces[i], 1), 1)/2;
			// double Fz = (model->vertices_update_(model->faces_(model->visible_faces[i], 0), 2) + model->vertices_update_(model->faces_(model->visible_faces[i], 2), 2)) / 4 + model->vertices_update_(model->faces_(model->visible_faces[i], 1), 2)/2;

			// glLineWidth(1);
			// glColor3f(0.0, 0.0, 1.0);
			// glBegin(GL_LINES);
			// glVertex3f(Fx, Fy, Fz);
			// glVertex3f(Fx + 5*model->faces_normals(model->visible_faces[i],0), Fy + 5*model->faces_normals(model->visible_faces[i], 1), Fz + 5*model->faces_normals(model->visible_faces[i], 2));
			// glEnd();

			// 

			// glLineWidth(1);
			// glColor3f(0.0, 1.0, 0);
			// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			// glFrontFace(GL_CCW);              // 设置逆时针方向为正面 
			// glBegin(GL_TRIANGLES);
			// glVertex3f(model->vertices_update_(model->faces_(model->visible_faces[i], 0), 0), model->vertices_update_(model->faces_(model->visible_faces[i], 0), 1), model->vertices_update_(model->faces_(model->visible_faces[i], 0), 2));
			// glVertex3f(model->vertices_update_(model->faces_(model->visible_faces[i], 1), 0), model->vertices_update_(model->faces_(model->visible_faces[i], 1), 1), model->vertices_update_(model->faces_(model->visible_faces[i], 1), 2));
			// glVertex3f(model->vertices_update_(model->faces_(model->visible_faces[i], 2), 0), model->vertices_update_(model->faces_(model->visible_faces[i], 2), 1), model->vertices_update_(model->faces_(model->visible_faces[i], 2), 2));
			// glEnd();

			//}


		}
		//glEnable(GL_LIGHTING);
		if (config.show_skeleton) {
			for (int i = 0; i < _data.joints.rows(); i++) {
				//画点开始
				glColor3f(1.0, 0.0, 0.0);
				glPushMatrix();
				glTranslatef(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
				glutSolidSphere(5, 31, 10);
				glPopMatrix();
				//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。

				//画线开始
				if (i != 0) {
					glLineWidth(5);
					glColor3f(0.0, 1.0, 0);
					glBegin(GL_LINES);
					int ii = _data.joints(i, 3);
					glVertex3f(_data.joints(ii, 0), _data.joints(ii, 1), _data.joints(ii, 2));
					glVertex3f(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
					glEnd();
				}

				//画线结束
			}
		}


		//画深度图转换成的点云的点
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < _cloudpoint.num_cloudpoint; i++)
		{
			glVertex3d(_cloudpoint.cloudpoint[i*3], _cloudpoint.cloudpoint[i*3 + 1], _cloudpoint.cloudpoint[i*3 + 2]);
		}
		glEnd();


		for (int i = 0; i < _cloudpoint.num_cloudpoint; i++)
		{

			glLineWidth(1);
			glColor3f(1.0, 0.0, 0);
			glBegin(GL_LINES);
			glVertex3f(_cloudpoint.cloudpoint[i * 3], _cloudpoint.cloudpoint[i * 3 + 1], _cloudpoint.cloudpoint[i * 3 + 2]);
			glVertex3f(_cloudpoint.cloudpointTomesh_inscribePoint[i * 3], _cloudpoint.cloudpointTomesh_inscribePoint[i * 3 + 1], _cloudpoint.cloudpointTomesh_inscribePoint[i * 3 + 2]);
			glEnd();

		}


		//画世界坐标系中的xyz三个轴
		glLineWidth(5);
		glColor3f(1.0, 0.0, 0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(1000, 0, 0);
		glEnd();

		glLineWidth(5);
		glColor3f(0.0, 1.0, 0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1000, 0);
		glEnd();

		glLineWidth(5);
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, -1000);
		glVertex3f(0, 0, 0);
		glEnd();

		
		glFlush();
		glutSwapBuffers();
	}


	void mouseMotion(int x, int y) {
		control.rotx = (x - control.x)*0.05;
		control.roty = (y - control.y)*0.05;


		control.gx = model->get_global_position().x;
		control.gy = model->get_global_position().y;
		control.gz = model->get_global_position().z;

		double r = 200;
		double xx = r*sin(control.roty)*cos(control.rotx);
		double yy = r*sin(control.roty)*sin(control.rotx);
		double zz = r*cos(control.roty);

		cameraPos.eye = cv::Point3f(xx + control.gx, yy + control.gy, zz + control.gz);
		cameraPos.center = cv::Point3f(control.gx, control.gy, control.gz);
		cameraPos.up = cv::Point3f(0.0f, 1.0f, 0.0f);
		//cout<< control.rotx <<" " << control.roty << endl;
		glutPostRedisplay();
	}

	/* executed when program is idle */
	void idle() {

        //neet to edit 如何调整手部参数
		//if (!_handcontrol->ParamsChangeStop)
		//{
		//	_handcontrol->ComputeGradient();
		//	_handcontrol->ParamsChangeUseGradient();


		//	//_handcontrol->ParamsChangeUseGaussNewTon();

		//	_handcontrol->ControlHand();

		//	SS::SubdivisionTheHand(model, 0);
		//	_cloudpoint.Compute_Cloud_to_Mesh_Distance();

		//	//cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);
		//	cv::Mat generated_mat = cv::Mat::zeros(_handcontrol->_costfunction.ROI_len_y, _handcontrol->_costfunction.ROI_len_x, CV_16UC1);
		//	projection->compute_current_orientation(model);
		//	projection->project_3d_to_2d_when_calc(model, generated_mat);
		//	MixShowResult(_handcontrol->_costfunction.groundtruthROIMat, generated_mat);

		//	_handcontrol->_costfunction.ComputeCostfunction(generated_mat);
		//	cout << "The "<<++i<<" th change !  The costfuntion is :" << _handcontrol->_costfunction.costfunction << endl;
		//	_data.init(SS::disVertices.size(), SS::disPatches.size());
		//	_data.SS_set(SS::disVertices, SS::disPatches);
		//	_data.set_skeleton(model);
		//	
		//}
		//else
		//{
		//	cout << "the costfunction meet the threshold, optimizing over ! "<<endl;
		//}

	    glutPostRedisplay();
	}

	/* initialize OpenGL settings */
	void initGL(int width, int height) {

		reshape(width, height);
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClearDepth(1.0f);
		cameraPos.eye = cv::Point3f(0.0f, 0.0f, model->get_global_position().z + 200);
		cameraPos.center = cv::Point3f(model->get_global_position().x, model->get_global_position().y, model->get_global_position().z);
		cameraPos.up = cv::Point3f(0, 1.0f, 0);
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);

	}


	void init(int argc, char* argv[]) {
		//glut简单教程，以下的函数都有提到：http://www.cnblogs.com/yangxi/archive/2011/09/16/2178479.html
		// 初始化GLUT
		glutInit(&argc, argv);

		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
		glutInitWindowSize(800, 600);
		glutInitWindowPosition(400, 200);
		glutCreateWindow("Interactron");

		initGL(800, 600);
		// register glut call backs
		glutKeyboardFunc(keyboardDown);
		glutKeyboardUpFunc(keyboardUp);
		glutSpecialFunc(keyboardSpecialDown);
		glutSpecialUpFunc(keyboardSpecialUp);
		glutMouseFunc(mouseClick);
		glutMotionFunc(mouseMotion);
		glutReshapeFunc(reshape);  //当使用鼠标改变opengl显示窗口时，被调用的函数，保证不变形
		glutDisplayFunc(draw);
		glutIdleFunc(idle);
		glutIgnoreKeyRepeat(true); // ignore keys held down
        // create a sub menu 
		int subMenu = glutCreateMenu(menu);
		glutAddMenuEntry("Do nothing", 0);
		glutAddMenuEntry("Really Quit", 'q');

		// create main "right click" menu
		glutCreateMenu(menu);
		glutAddSubMenu("Sub Menu", subMenu);
		glutAddMenuEntry("Quit", 'q');
		glutAttachMenu(GLUT_RIGHT_BUTTON);

		
	}

	void start() {
		// 通知开始GLUT的内部循环
		glutMainLoop();
	}


}




void MixShowResult(cv::Mat input1, cv::Mat input2)
{
	int height = input2.rows;
	int width = input2.cols;
	cv::Mat colored_input1 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat colored_input2 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat dst;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (input1.at<ushort>(i, j) != 0)
			{
				colored_input1.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;
			}
			else
			{

				colored_input1.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;

			}

			if (input2.at<ushort>(i, j) != 0)
			{
				colored_input2.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{

				colored_input2.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 255;

			}

		}
	}

	cv::addWeighted(colored_input1, 0.5, colored_input2, 0.5, 0.0, dst);
	cv::imshow("Mixed Result", dst);

}