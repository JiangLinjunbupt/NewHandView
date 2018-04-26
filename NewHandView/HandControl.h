#pragma once

#include "Model.h"
#include "Random.h"

class RotateControl
{
private:
	float RotateX;
	float RotateY;
	float RotateZ;

public:
	RotateControl() { this->RotateX = 0; this->RotateY = 0; this->RotateZ = 0; }
	~RotateControl() {}

	float GetRotateX() { return this->RotateX; }
	float GetRotateY() { return this->RotateY; }
	float GetRotateZ() { return this->RotateZ; }
	void SetRotateX(float x) { this->RotateX = x; }
	void SetRotateY(float y) { this->RotateY = y; }
	void SetRotateZ(float z) { this->RotateZ = z; }
	void SetRotateXYZ(float x, float y, float z) { this->RotateX = x; this->RotateY = y; this->RotateZ = z; }
};

class TransControl
{
private:
	float transX;
public:
	TransControl() {this->transX = 0; }
	~TransControl() {}

	float GettransX() { return this->transX; }
	void SettransX(float x) { this->transX = x; }

};

class Finger {
private:
	float fingerscale;
	RotateControl *rotation;
	TransControl *trans;
public:
	Finger() { this->fingerscale = 1; this->rotation = new RotateControl[4]; this->trans = new TransControl[4]; }
	~Finger() { delete rotation; delete trans; }

	float Getfingerscale() { return this->fingerscale; }
	RotateControl* GetRotete() { return this->rotation; }
	TransControl* GetTrans() { return this->trans; }

	void Setfingerscale(float scale) { this->fingerscale = scale; }
	void SetRotate(RotateControl R,int index) { this->rotation[index] = R; }
	void SetRotate(float x, float y, float z, int index) { this->rotation[index].SetRotateXYZ(x, y, z); }
	void SetTrans(TransControl T,int index) { this->trans[index] = T; }
	void SetTrans(float t, int index) { this->trans[index].SettransX(t); }
};

class Palm {
private:
	float palmscale;
	RotateControl rotation;
public:
	Palm() { this->palmscale = 1;}
	~Palm() {}

	float Getplamscale() { return this->palmscale; }
	RotateControl GetRotate() { return this->rotation; }

	void Setpalmscale(float scale) { this->palmscale = scale; }
	void SetRotate(RotateControl R) { this->rotation = R; }
	void SetRotate(float x, float y, float z) { this->rotation.SetRotateXYZ(x, y, z); }
};
struct HandControl
{

	Finger *fingers;
	Palm palm;
	Pose palm_position;
	
	HandControl() { this->fingers = new Finger[5]; palm_position.x = 0, palm_position.y = 0, palm_position.z = -800; }
	~HandControl() { delete this->fingers; }
	void SetPlam_Position(float x, float y, float z)
	{
		Pose p;
		p.x = x; p.y = y; p.z = z;
		this->palm_position = p;
	}
	void SetPlam(Palm p) { this->palm = p; }
	void SetFingers(Finger* f) { this->fingers = f;}

	void ControlHand() {
		model->set_global_position(palm_position);
		model->set_global_position_center(palm_position);

		model->set_joint_scale(palm.Getplamscale(), 0);
		model->set_joint_scale(palm.Getplamscale(), 21);  //这里让手腕和手掌同等缩小放大，防止看起来怪异
		model->set_one_rotation(Pose(palm.GetRotate().GetRotateX(), palm.GetRotate().GetRotateY(), palm.GetRotate().GetRotateZ()),0);


		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				model->set_joint_scale(fingers[i].Getfingerscale(), i*4+j+1);
				model->set_one_rotation(Pose(fingers[i].GetRotete()[j].GetRotateX(), fingers[i].GetRotete()[j].GetRotateY(), fingers[i].GetRotete()[j].GetRotateZ()), i*4 + j + 1);
				model->set_one_trans(fingers[i].GetTrans()[j].GettransX(), i*4 + j + 1);
			}
		}
	}

	void RandomGenerateParams()
	{
		Pose upper, lower;
		Random random;
		//随机全局位置
		float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-200, 0);
		random_palm_positonY = random.Next(-200, 0);
		random_palm_positonZ = random.Next(-800, -400);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);

		//随机手掌参数
		Palm random_palm;
		float random_palmscale;
		random_palmscale = 1+ random.NextDouble();   //[0,2]
		random_palm.Setpalmscale(random_palmscale);
		RotateControl random_palmRotation;
		upper = model->get_upper_of_angle(0);
		lower = model->get_lower_of_angle(0);
		random_palmRotation.SetRotateX(random.Next(lower.x, upper.x));
		random_palmRotation.SetRotateY(random.Next(lower.y, upper.y));
		random_palmRotation.SetRotateZ(random.Next(lower.z, upper.z));
		random_palm.SetRotate(random_palmRotation);
		this->SetPlam(random_palm);

		//随机手指参数
		Finger *random_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float random_fingerscale;
			random_fingerscale = 1+ 0.5*random.NextDouble();
			random_fingers[i].Setfingerscale(random_fingerscale);
			for (int j = 0; j < 4; j++)
			{
				RotateControl random_fingerRotation;
				upper = model->get_upper_of_angle(i * 4 + j + 1);
				lower = model->get_lower_of_angle(i * 4 + j + 1);
				random_fingerRotation.SetRotateX(random.Next(lower.x, upper.x));
				random_fingerRotation.SetRotateY(random.Next(lower.y, upper.y));
				random_fingerRotation.SetRotateZ(random.Next(lower.z, upper.z));

				TransControl random_fingerTrans;
				if (j == 3)
				{
					random_fingerTrans.SettransX(random.Next(0, 0));       //这里发现指尖点对mesh顶点是没有权重的，因此，手指的第三个关节长度是没办法变化的。
				}
				else
				{
					random_fingerTrans.SettransX(random.Next(10, 30));     //
				}

				random_fingers[i].SetRotate(random_fingerRotation, j);
				random_fingers[i].SetTrans(random_fingerTrans, j);
			}
		}
		this->SetFingers(random_fingers);


	}

	void RandomScaleAndTransParams()
	{
		Random random;
		//随机全局位置
		float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-200, 0);
		random_palm_positonY = random.Next(-200, 0);
		random_palm_positonZ = random.Next(-800, -400);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);

		//随机手掌参数
		Palm random_palm;
		float random_palmscale;
		random_palmscale = 1 + random.NextDouble();   //[1,2]
		random_palm.Setpalmscale(random_palmscale);
		RotateControl random_palmRotation;
		random_palmRotation.SetRotateX(this->palm.GetRotate().GetRotateX());
		random_palmRotation.SetRotateY(this->palm.GetRotate().GetRotateY());
		random_palmRotation.SetRotateZ(this->palm.GetRotate().GetRotateZ());
		random_palm.SetRotate(random_palmRotation);
		this->SetPlam(random_palm);

		//随机手指参数
		Finger *random_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float random_fingerscale;
			random_fingerscale = 1 + 0.5*random.NextDouble();
			random_fingers[i].Setfingerscale(random_fingerscale);
			for (int j = 0; j < 4; j++)
			{
				RotateControl random_fingerRotation;
				random_fingerRotation.SetRotateX(this->fingers[i].GetRotete()[j].GetRotateX());
				random_fingerRotation.SetRotateY(this->fingers[i].GetRotete()[j].GetRotateY());
				random_fingerRotation.SetRotateZ(this->fingers[i].GetRotete()[j].GetRotateZ());

				TransControl random_fingerTrans;
				if (j == 3)
				{
					random_fingerTrans.SettransX(random.Next(0, 0));       //这里发现指尖点对mesh顶点是没有权重的，因此，手指的第三个关节长度是没办法通过这个关节变化的，只能通过其他关节变化。
				}
				else
				{
					random_fingerTrans.SettransX(random.Next(10, 30));     //
				}

				random_fingers[i].SetRotate(random_fingerRotation, j);
				random_fingers[i].SetTrans(random_fingerTrans, j);
			}
		}
		this->SetFingers(random_fingers);
	}

	void SaveParams(char* filename){
		ofstream f;
		f.open(filename, ios::out);
		f << this->palm_position.x << " " << this->palm_position.y <<" "<<this->palm_position.z<< endl;

		f << this->palm.Getplamscale() << " " << this->palm.GetRotate().GetRotateX() << " " << this->palm.GetRotate().GetRotateY() << " " << this->palm.GetRotate().GetRotateZ() << endl;
		for (int i = 0; i < 5; i++) {
			
			f << this->fingers[i].Getfingerscale() << endl;
			for (int j = 0; j < 4; j++)
			{
				f << this->fingers[i].GetRotete()[j].GetRotateX() << " " << this->fingers[i].GetRotete()[j].GetRotateY() << " " << this->fingers[i].GetRotete()[j].GetRotateZ() << " "<< this->fingers[i].GetTrans()[j].GettransX() <<endl;
			}
		}
		f.close();
	}

	void LoadParams(char* filename){
		fstream f;
		f.open(filename, ios::in);
		f >> this->palm_position.x>> this->palm_position.y>> this->palm_position.z;

		float read_palmscale, read_RotateX, read_RotateY, read_RotateZ;
		RotateControl read_rotate;
		f >> read_palmscale >> read_RotateX >> read_RotateY >> read_RotateZ;

		read_rotate.SetRotateX(read_RotateX);
		read_rotate.SetRotateY(read_RotateY);
		read_rotate.SetRotateZ(read_RotateZ);
		this->palm.Setpalmscale(read_palmscale);
		this->palm.SetRotate(read_rotate);

		Finger *read_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float read_fingerscale;
			f >> read_fingerscale;
			read_fingers[i].Setfingerscale(read_fingerscale);
			for (int j = 0; j < 4; j++) {
				RotateControl read_fingerRotation;
				TransControl read_fingerTrans;
				float read_fingerRotateX, read_fingerRotateY, read_fingerRotateZ, read_fingertrans;
				f >> read_fingerRotateX >> read_fingerRotateY >> read_fingerRotateZ>> read_fingertrans;
				read_fingerRotation.SetRotateX(read_fingerRotateX);
				read_fingerRotation.SetRotateY(read_fingerRotateY);
				read_fingerRotation.SetRotateZ(read_fingerRotateZ);
				read_fingerTrans.SettransX(read_fingertrans);

				read_fingers[i].SetRotate(read_fingerRotation, j);
				read_fingers[i].SetTrans(read_fingerTrans, j);

			}
		}
		this->SetFingers(read_fingers);

		f.close();
	}


};


static HandControl handcontrol;