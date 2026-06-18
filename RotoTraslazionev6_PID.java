package org.firstinspires.ftc.kikko;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "RotoTraslazionev6_PID")
public class RotoTraslazionev6_PID extends LinearOpMode {

  private DcMotor sud;
  private DcMotor east;
  private DcMotor ovest;
  private IMU imu;
  
  boolean verboso = true;                //scegli quanto far parlare il programma (true tanto; false poco)
  
  double PS, PE, PO;                      //potenza del Motore Sud, East, Ovest
  double[] P = new double[3];             //array potenze
  double p = 0.4;                         //fattore attenuazine della potenza dei motori
  double p_i = p;                         //fattore attenuazine della potenza dei motori
  
  double G = Math.toRadians(0);           //angolo di inclinazione del Motore Sud rispetto all'asse X               (0)
  double cosG = Math.cos(G);              //coseno dell'angolo di inclinazione del Motore Sud rispetto all'asse X   (1)
  double sinG = Math.sin(G);              //seno dell'angolo di inclinazione del Motore Sud rispetto all'asse X     (0)
  
  double A = Math.toRadians(120);         //angolo di inclinazione del Motore East rispetto all'asse X              (120)
  double cosA = Math.cos(A);              //coseno dell'angolo di inclinazione del Motore East rispetto all'asse X  (-1/2)
  double sinA = Math.sin(A);              //seno dell'angolo di inclinazione del Motore East rispetto all'asse X    (√3/2)
  
  double B = Math.toRadians(-120);        //angolo di inclinazione del Motore Ovest rispetto all'asse X             (-120)
  double cosB = Math.cos(B);              //coseno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X (-1/2)
  double sinB = Math.sin(B);              //seno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X   (-√3/2)
  
  ElapsedTime cronometro = new ElapsedTime();
  ElapsedTime timer = new ElapsedTime();
  
  double errore = 0;
  double errore_precedente = 0;
  double tempo = 0;
  double tempo_precedente = 0;
  double delta_tempo = 0;
  double integrale = 0;
  double derivata = 0;
  double output = 0;
  
  double phi;
  double theta;                           //angolo di inclinazione del robot rispetto all'asse z (giroscopio)
  double omega;
  
  int type = 0;                           //Tipo di PID
  
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  
  private ArrayList<String> logBuffer = new ArrayList<>();
  String nomefile;
  
  @Override
  public void runOpMode() {
    // Definiamo il giroscopio e l'orientazione del controller hub
    YawPitchRollAngles orientation;
    imu = hardwareMap.get(IMU.class, "imu");
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    
    AngularVelocity velAngolare;
    
    // Definiamo i 3 motori
    sud = hardwareMap.get(DcMotor.class, "sud");
    east = hardwareMap.get(DcMotor.class, "east");
    ovest = hardwareMap.get(DcMotor.class, "ovest");
    
    // Dichiariamo che utilizziamo i motori con l'utilizzo con l'encoder
    sud.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    east.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ovest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    // Quando Potenza = 0 il motore è frenato
    sud.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    east.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    
    ovest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //Definiamo la direzione dei motori
    sud.setDirection(DcMotorSimple.Direction.REVERSE); //è al contrario
    east.setDirection(DcMotorSimple.Direction.FORWARD);
    ovest.setDirection(DcMotorSimple.Direction.FORWARD);
    
    // Impostiamo i coeficienti per utilizzare il motore in modalità encoder
    ((DcMotorEx) sud).setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
    ((DcMotorEx) east).setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
    ((DcMotorEx) ovest).setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
    
    telemetry.addData("Sto", "attendendo lo start...");
    telemetry.update();
    
    nomefile = "Movimento_PIDDATO.txt";
    waitForStart();
    if (opModeIsActive()) {
      imu.resetYaw();
      cronometro.reset();
      timer.reset();
      telemetry.update();
      
      double Xl, Yl;                          //Valore X, Y Stick Sinistra
      double Xr, Yr;                          //Valore X, Y Stick Destra
      
      phi = 0;
      
      int state = 0;                          //Tipo di moto
      String moto_attuale = "default";
      String PID_attuale = "default";
      
      while (opModeIsActive()) {
        orientation = imu.getRobotYawPitchRollAngles();
        velAngolare = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        telemetry.update();
        if (gamepad1.right_stick_button) {imu.resetYaw(); reset_PID();}                         //R3 reset giroscopio
        if (gamepad1.rightBumperWasPressed()) {state += 1; if (state > 3){state = 3;}}          //R1 aumenta stato
        if (gamepad1.leftBumperWasPressed()) {state += -1; if (state < 0){state = 0;}}          //L1 diminuisci stato
        if (gamepad1.ps) {phi = 0;}                                                             //PS torna a stato default
        if (gamepad1.dpadUpWasPressed()) {reset_PID(); p += 0.1; if (p > 0.6){p = 0.6;}}        //Freccia su aumenta potenza
        if (gamepad1.dpadDownWasPressed()) {reset_PID(); p += -0.1; if (p < 0.3){p = 0.3;}}     //Freccia giù diminuisci potenza
        if (gamepad1.dpadRightWasPressed()) {reset_PID(); type += 1; if (type > 1){type = 1;}}  //Freccia destra cambia PID
        if (gamepad1.dpadLeftWasPressed()) {reset_PID(); type -= 1; if (type < -1){type = -1;}} //Freccia sinistra cambia PID
        if (gamepad1.start) {reset_PID(); p = p_i;}                                                          //START reset potenza
        
        Xl =  (gamepad1.left_stick_x);
        Yl =  -(gamepad1.left_stick_y);
        Xr =  -(gamepad1.right_stick_y);
        Yr =  -(gamepad1.right_stick_x);
        phi = find_phi(Xr, Yr);
        theta = orientation.getYaw(AngleUnit.RADIANS);
        omega = velAngolare.zRotationRate;
        switch (type){
          case -1:
            PID_attuale = "PID_w";
          break;
          case 0:
            PID_attuale = "no PID";
          break;
          case 1:
            PID_attuale = "PID_t";
          break;
        }
        
        switch (state){
          case 0:
            moto_attuale = "Rototraslazione";
            P = rototraslazione(Xl, Yl, omega, phi, theta);
          break;
          case 1:
            moto_attuale = "Traslazione";
            P = traslazione(Xl, Yl);
          break;
          case 2:
            moto_attuale = "Rotazione";
            P = rotazione_scomposta(Xr, Yr, theta);
          break;
        }
        P = normalizzazione(P);
        PS = P[0];
        PE = P[1];
        PO = P[2];
        sud.setPower(PS);
        east.setPower(PE);
        ovest.setPower(PO);
        raccolta_dati(0, omega, phi, theta);
        
        if (verboso) {
          telemetry.addData("moto: ", moto_attuale);
          telemetry.addData("PID: ", PID_attuale);
          telemetry.addData("p", p);
          telemetry.addData("xr", Xr);
          telemetry.addData("yr", Yr);
          telemetry.addData("xl", Xl);
          telemetry.addData("yl", Yl);
          telemetry.addData("PS", PS);
          telemetry.addData("PE", PE);
          telemetry.addData("PO", PO);
          telemetry.addData("tempo", tempo);
          telemetry.addData("theta", Math.toDegrees(theta));
          telemetry.addData("phi", Math.toDegrees(phi));
          telemetry.addData("omega", Math.toDegrees(omega));
          telemetry.addData("errore", Math.toDegrees(errore));
          telemetry.addData("delta tempo", delta_tempo);
        }
      }
      if (!logBuffer.isEmpty()) {ScriviBufferSuFile(nomefile, logBuffer);}
    }
    
    if (!logBuffer.isEmpty()) {
      telemetry.addData("Sto", "scrivendo sul log...");
      telemetry.update();
      ScriviBufferSuFile(nomefile, logBuffer);
      sleep(10000);
      telemetry.addData("Sto", "chiudendo tutto...");
      telemetry.update();
      sleep(1000);
    }
  }
  
  private double[] cambio_coordinate(double x, double y, double alpha) {
    double[] cambio = new double[6];
    double[] coordinate_finali = new double[3];
    double cosa = Math.cos(alpha);
    double sina = Math.sin(alpha);
    //Matrice cambio coordinate
    cambio[0] = cosG*cosa-sinG*sina;
    cambio[1] = sinG*cosa+cosG*sina;
    cambio[2] = cosA*cosa-sinA*sina;
    cambio[3] = sinA*cosa+cosA*sina;
    cambio[4] = cosB*cosa-sinB*sina;
    cambio[5] = sinB*cosa+cosB*sina;
    
    for (int i = 0; i < 3; i++) {coordinate_finali[i] = x * cambio[i * 2] + y * cambio[i * 2 + 1];}
    return coordinate_finali;
  }
  private double find_phi(double x, double y) {
    if(x == 0 && y == 0){return 0;}
    else{return Math.atan2(y, x);}
  }
  private double[] normalizzazione(double[] vettore) {
    double max = Math.max(Math.abs(vettore[0]), Math.max(Math.abs(vettore[1]), Math.abs(vettore[2])));
    if (max > p) {for (int i = 0; i < 3; i++) {vettore[i] /= max/p;}}
    return vettore;
  }
  
  private double[] traslazione(double x, double y) {
    double[] potenze_traslazionali = new double[3];
    potenze_traslazionali = cambio_coordinate(x, y, 0);
    return potenze_traslazionali;
  }
  private double[] rotazione_scomposta(double x, double y, double beta) {//x,y richiesto, beta letto
    double[] potenze_rotazionali = new double[3];
    double cost = Math.cos(beta);
    double sint = Math.sin(beta);
    double r = Math.hypot(x, y);
    for (int i = 0; i < 3; i++) {potenze_rotazionali[i] = r * (y*cost - x*sint);}
    return potenze_rotazionali;
  }
  private double[] rotazione(double differenza) {
    double[] potenze_rotazionali = new double[3];
    for (int i = 0; i < 3; i++) {potenze_rotazionali[i] = differenza;}
    return potenze_rotazionali;
  }
  private double[] rototraslazione(double x, double y, double omega, double alpha, double beta) {
    double[] traslazionali = new double[3];
    double[] rotazionali = new double[3];
    double[] potenze_rototraslazionali = new double[3];
    traslazionali = traslazione(x, y);
    rotazionali = rotazione(PID(0, omega, alpha, beta));
    for (int i = 0; i < 3; i++) {potenze_rototraslazionali[i] = traslazionali[i] + rotazionali[i];}
    return potenze_rototraslazionali;
  }
  
  private double PID(double riferimento_w, double lettura_w, double riferimento_t, double lettura_t){
    PID_fit();
    switch (type){
      case -1:
        errore = riferimento_w - lettura_w;
        integrale = riferimento_t - lettura_t;
        tempo = cronometro.seconds();
        delta_tempo = tempo - tempo_precedente;
        if (delta_tempo <= 0) {delta_tempo = 0.001;}
      break;
      case 0:
        errore = riferimento_t - lettura_t;
        tempo = cronometro.seconds();
        delta_tempo = tempo - tempo_precedente;
        if (delta_tempo <= 0) {delta_tempo = 0.001;}
        integrale += errore * delta_tempo;
      break;
      case 1:
        errore = riferimento_t - lettura_t;
        tempo = cronometro.seconds();
        delta_tempo = tempo - tempo_precedente;
        if (delta_tempo <= 0) {delta_tempo = 0.001;}
        integrale += errore * delta_tempo;
      break;
    }
    if (errore > Math.PI) {errore -= 2 * Math.PI;}
    else if (errore < -Math.PI) {errore += 2 * Math.PI;}
    derivata = (errore - errore_precedente)/delta_tempo; //approssimazione
    errore_precedente = errore;
    tempo_precedente = tempo;
    output = (errore * Kp) + (integrale * Ki) + (derivata * Kd);
    return output;
  }
  private void reset_PID(){
    errore_precedente = 0;
    integrale = 0;
    derivata = 0;
  }
  private void PID_fit(){
    switch (type){
      case -1:
        if (p == 0.4){Kp = 0.3733021196832707; Ki = 1.6004655198721427; Kd = 0.013102590620354728;}
        else{Kp =  0; Ki =  0; Kd =  0;}
      break;
      case 0:
        Kp =  0; Ki =  0; Kd =  0;
      break;
      case 1:
        if (p == 0.6){Kp = 0.88362; Ki = 2.2505991596539565; Kd = 0.08673071580192564;}
        else if (p == 0.5){Kp = 1.3499999999999999; Ki = 3.2730709864795053; Kd = 0.1392041302746285;}
        else if (p == 0.4){Kp = 1.3499999999999999; Ki = 3.7105538480072124; Kd = 0.12279164207378301;}
        else if (p == 0.3){Kp = 1.68; Ki = 5.95936795516598; Kd = 0.11840181799620855;}
        else{Kp =  0; Ki =  0; Kd =  0;}
      break;
    }
  }
  
  private File fileLog;
  private PrintWriter scrittoreFile;
  
  private void raccolta_dati(double w_rif, double w_let, double alpha, double beta) {
    String riga = String.format(java.util.Locale.US, "%.4f  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f", cronometro.seconds(), w_rif, w_let, w_rif - w_let, alpha, beta, alpha - beta, output);
    logBuffer.add(riga);
  }
  
  private void ScriviBufferSuFile(String nomeFile, ArrayList<String> buffer) {
    try {
      File directory = new File("/sdcard/FIRST/data/");
      if (!directory.exists()) {directory.mkdirs();} 
      File file = new File(directory, nomeFile);
      
      PrintWriter scrittore = new PrintWriter(new FileWriter(file, false)); 
      for (String linea : buffer) {scrittore.println(linea);}
      scrittore.flush();
      scrittore.close();
    }
    catch (IOException e) {}
  }
}
