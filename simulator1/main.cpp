//
//  main.cpp
//  simulator1
//
//  Created by 上田　護 on 2018/03/20.
//  Copyright © 2018年 上田　護. All rights reserved.
//

#include <iostream>

//int main(int argc, const char * argv[]) {
//    // insert code here...
//    std::cout << "Hello, World!\n";
//    return 0;
//}
//#include"stdafx.h"
#include<stdio.h>
#include<math.h>

#define _USE_MATH_DEFINES
#define M_PI 3.14


//ロボットのパラメータ
double m[2]  = { 5.0, 5.0 };        //質量
double l[2]  = { 0.5, 0.5 };        //リンクの長さ
double lc[2] = { 0.0, 0.0 };        //重心位置
double I[2]  = { 0.0, 0.0 };        //慣性モーメント
double g = 9.8;                        //重力定数

//ゲイン
double kp[2]    = { 0.0, 0.0 };        // 位置フィードバック
double kv[2]    = { 0.0, 0.0 };        // 速度フィードバックゲイン
double ki[2]    = { 0.0, 0.0 };        // 積分フィードバックゲイン
double sum_q[2] = { 0.0, 0.0 };        // 偏差積分項(関節角度)
double sum_x[2] = { 0.0, 0.0 };        // 偏差積分項(手先位置)

//角度
double q[2]         = { 0.0, 0.0 };    //関節角度
double dot_q[2]     = { 0.0, 0.0 };    //関節角速度
double ddot_q[2] = { 0.0, 0.0 };    //関節角加速度

//目標角度，角速度
double qd[2]     = { 0.0, 0.0 };    //目標角度
double dot_qd[2] = { 0.0, 0.0 };    //目標角速度

//手先位置
double X  = 0.0;                    //手先位置x座標
double Y  = 0.0;                    //手先位置y座標

//目標手先位置
double Xd = 30.0;                     //目標手先位置x座標
double Yd = 0.0;                     //目標手先位置y座標

//ダイナミクス
double M[4]        = { 0.0 };    //慣性項
double detM        = 0.0;        //逆行列計算用行列式
double Minv[4]    = { 0.0 };    // 行列Mの逆行列
double h[2]        = { 0.0 };    //コリオリ力
double G[2]        = { 0.0 };    //重力項
double tau[2]    = { 0.0 };    //トルク
double Jt[4]    = { 0.0 };  //ヤコビ転置

//時間
double theTime      = 0.0;        //現在時間
double CT        = 10.0;        //制御時間
double ST        = 0.001;    //サンプリングタイム

//ファイル出力
FILE *fp;




int main(){
    //ファイル出力
//        fopen_s(&fp, "data.csv","w");
//        fprintf(fp, "Time[s],q[0]q[0],q[1],dot_q[0],dot_q[1],ddot_q[0],ddot_q[1],X,Y,Xd,Yd\n");
    
    //ルンゲクッタ法
    for(theTime=0.0; theTime<CT; theTime+=ST){
         printf("%f\n", q[0]);
        // オイラー法
        q[0] += dot_q[0] * ST;
        q[1] += dot_q[1] * ST;
        dot_q[0] += ddot_q[0] * ST;
        dot_q[1] += ddot_q[1] * ST;
        
        //ルンゲクッタ法
        //ルンゲクッタ法を記述して制御を行ってください
        //慣性項計算（各項を計算して入力してください）
        M[0] = 0.0;
        M[1] = 0.0;
        M[2] = 0.0;
        M[3] = 0.0;
        
        //行列Mの逆行列計算
        detM = M[0]*M[3] - M[1]*M[2];
        Minv[0] =        M[3] / detM;
        Minv[1] = -1.0 * M[2] / detM;
        Minv[2] = -1.0 * M[1] / detM;
        Minv[3] =        M[0] / detM;
        
        //コリオリ項計算（各項を計算して入力してください）
        h[0] = 0.0;
        h[1] = 0.0;
        
        //重力項計算（各項を計算して入力してください）
        G[0] = 0.0;
        G[1] = 0.0;
        
        //目標角度，角速度（関節角度制御の際に入力してください）
        qd[0]      = 0.0;
        qd[1]      = 0.0;
        dot_qd[0] = 0.0;
        dot_qd[1] = 0.0;
        
        //目標手先位置入力（手先の位置制御の際に入力してください）
        Xd = 0.0;
        Yd = 0.0;
        
        //ヤコビ転置（各項を計算して入力してください）
        Jt[0] = 0.0;
        Jt[1] = 0.0;
        Jt[2] = 0.0;
        Jt[3] = 0.0;
        
        //出力トルク計算(関節角度制御式入力)
        tau[0] = 0.0;
        tau[1] = 0.0;
        
        tau[0] = 0.0;
        tau[1] = 0.0;
        
        //関節角加速度計算(各項の計算式を入力してください)
        ddot_q[0] = 0.0;
        ddot_q[1] = 0.0;
        
        //関節角度偏差積分計算（各項の計算式を入力してください）
        sum_q[0] += 0.0;
        sum_q[1] += 0.0;
        
        //手先位置偏差積分計算（各項の計算式を入力してください）
        sum_x[0] += 0.0;
        sum_x[1] += 0.0;
        
        //手先位置計算（各項の計算式を入力してください）
        X = 0.0;
        Y = 0.0;
        
        //ファイル出力
        //        printf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",time,q[0],q[1],dot_q[0],dot_q[1],ddot_q[0],ddot_q[1],X,Y,Xd,Yd);
        
    }
    
    fclose(fp);
}


