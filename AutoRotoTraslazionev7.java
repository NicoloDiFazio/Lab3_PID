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

@Autonomous(name = "AutoRotoTraslazionev7", group = "Autonomous")
public class AutoRotoTraslazionev7 extends LinearOpMode {

  private DcMotor sud;
  private DcMotor east;
  private DcMotor ovest;
  private IMU imu;
  
  boolean verboso = false;                 //scegli quanto far parlare il programma (true tanto; false poco)
  
  double PS, PE, PO;                      //potenza del Motore Sud, East, Ovest
  double[] P = new double[3];             //array potenze
  double p = 0.5;                           //fattore attenuazine della potenza dei motori
  
  double G = Math.toRadians(0);               //angolo di inclinazione del Motore Sud rispetto all'asse X               (0)
  double cosG = Math.cos(G);                  //coseno dell'angolo di inclinazione del Motore Sud rispetto all'asse X   (1)
  double sinG = Math.sin(G);                  //seno dell'angolo di inclinazione del Motore Sud rispetto all'asse X     (0)
  
  double A = Math.toRadians(120);             //angolo di inclinazione del Motore East rispetto all'asse X              (120)
  double cosA = Math.cos(A);                  //coseno dell'angolo di inclinazione del Motore East rispetto all'asse X  (-1/2)
  double sinA = Math.sin(A);                  //seno dell'angolo di inclinazione del Motore East rispetto all'asse X    (√3/2)
  
  double B = Math.toRadians(-120);            //angolo di inclinazione del Motore Ovest rispetto all'asse X             (-120)
  double cosB = Math.cos(B);                  //coseno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X (-1/2)
  double sinB = Math.sin(B);                  //seno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X   (-√3/2)
  
  
  ElapsedTime cronometro = new ElapsedTime(); //cronometro totale, tiene il tempo di tutto il programma
  ElapsedTime timer = new ElapsedTime();      //timer, da usare per tenere il tempo di un periodo preciso
  
  double errore = 0;
  double errore_precedente = 0;
  double tempo = 0;
  double tempo_precedente = 0;
  double delta_tempo = 0;
  double integralSum = 0;
  double derivata = 0;
  double output = 0;
  
  double phi = 0;
  
  double Kp = 2.7253345222602285; //0.5318442383620181;;
  double Ki = 74.74124818676849; //2; //2.253450776719937;
  double Kd = 0; //0.1; //0.022039030844941594;
  
  private ArrayList<String> logBuffer = new ArrayList<>();
  String nomefile;
  
  @Override
  public void runOpMode() {
    // Definiamo il giroscopio e l'orientazione del controller hub
    YawPitchRollAngles orientation;
    imu = hardwareMap.get(IMU.class, "imu");
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    orientation = imu.getRobotYawPitchRollAngles();
    
    AngularVelocity velAngolare;
    velAngolare = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
    //double w = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
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
    
    waitForStart();
    if (opModeIsActive()) {
      imu.resetYaw();
      cronometro.reset();
      timer.reset();
      telemetry.addData("Sto", "eseguendo il programma...");
      telemetry.update();
      
      phi = 0;                         //angolo di inclinazione richiesto
      double d_a = 45;                         //passo angolo
      double d_ar = Math.toRadians(d_a);
      
      //void moto_temporale(int moto, double tempo, double x, double y, double alpha)
      
      //moto_temporale(-1, 5, 0, 0, phi);
      //moto_temporale(0, 0.1, 0, 0, phi+d_ar);
      //moto_temporale(0, 20, 0, 0, phi);
      
      //moto_temporale(2, 3, 0, 0.5, phi);
      //moto_temporale(-1, 2, 0, 0.5, phi);
      double timer = 4;
      moto_temporale(-1, 5, 0, 0, 0);
      moto_temporale(2, timer, 0, 0.5, 0);
      moto_temporale(-1, 5, 0, 0, 0);
      
      nomefile = String.valueOf(Kp) + "_" + String.valueOf(Ki) + "_" + String.valueOf(Kd) + "_" + String.valueOf(timer) + "_PID.txt";
    }
    moto_temporale(-1, 0.1, 0, 0, phi);
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
  private double[] azzera(double[] vettore){
    for (int i = 0; i < 3; i++) {vettore[i] = 0;}
    return vettore;
  }
  
  private double[] traslazione(double x, double y) {
    double[] potenze_traslazionali = new double[3];
    potenze_traslazionali = cambio_coordinate(x, y, 0);
    return potenze_traslazionali;
  }
  private double[] rotazione(double differenza) {
    double[] potenze_rotazionali = new double[3];
    for (int i = 0; i < 3; i++) {potenze_rotazionali[i] = differenza;}
    return potenze_rotazionali;
  }
  private void rotazione_impulsiva(double alpha) {
    double beta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    while(opModeIsActive() && alpha - beta > 0){
      errore = alpha - beta;
      output = p*Math.signum(errore);
      for (int i = 0; i < 3; i++) {P[i] = output;}
      PS = P[0];
      PE = P[1];
      PO = P[2];
      sud.setPower(PS);
      east.setPower(PE);
      ovest.setPower(PO);
      raccolta_dati(alpha, beta);
      beta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    errore = alpha - beta;
    output = -p;
    sud.setPower(output);
    east.setPower(output);
    ovest.setPower(output);
    raccolta_dati(alpha, beta);
  }
  private double[] rototraslazione(double x, double y, double alpha, double beta) {
    double[] traslazionali = new double[3];
    double[] rotazionali = new double[3];
    double[] potenze_rototraslazionali = new double[3];
    traslazionali = traslazione(x, y);
    rotazionali = rotazione(PID_w(alpha, beta));
    for (int i = 0; i < 3; i++) {potenze_rototraslazionali[i] = traslazionali[i] + rotazionali[i];}
    return potenze_rototraslazionali;
  }
  
  private void moto_temporale(int moto, double t, double x, double y, double alpha) {
    timer.reset();
    tempo_precedente = cronometro.seconds();
    integralSum = 0;
    errore_precedente = alpha - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
    while(opModeIsActive() && timer.seconds() <= t){
      double beta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);   //angolo di inclinazione del robot rispetto all'asse z (giroscopio)
      double omega = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
      switch (moto){
        case -1: P = azzera(P); output = 0; errore = alpha - omega; break; //"stop";
        case 0:  P = rotazione(alpha); output = alpha; break; //"Rotazione";
        case 1:  P = traslazione(x, y); output = 0; errore = alpha - beta; break; //"Traslazione";
        case 2:  P = rototraslazione(x, y, alpha, omega); break; //"Rototralsazione";
      }
      P = normalizzazione(P);
      PS = P[0];
      PE = P[1];
      PO = P[2];
      sud.setPower(PS);
      east.setPower(PE);
      ovest.setPower(PO);
      
      if (verboso) {
        telemetry.addData("moto", moto);
        telemetry.addData("PS", PS);
        telemetry.addData("PE", PE);
        telemetry.addData("PO", PO);
        telemetry.addData("tempo", cronometro.seconds());
        telemetry.addData("gyro", Math.toDegrees(beta));
        telemetry.addData("phi", Math.toDegrees(alpha));
        telemetry.addData("errore", Math.toDegrees(errore));
        telemetry.addData("delta tempo", delta_tempo);
        telemetry.update();
      }
      raccolta_dati(alpha, omega);
    }
  }
  
  private double PID(double riferimento, double lettura){
    errore = riferimento - lettura;
    if (errore > Math.PI) {errore -= 2 * Math.PI;}
    else if (errore < -Math.PI) {errore += 2 * Math.PI;}
    tempo = cronometro.seconds();
    delta_tempo = tempo - tempo_precedente;
    if (delta_tempo <= 0) {delta_tempo = 0.001;}
    integralSum += errore * delta_tempo;
    derivata = (errore - errore_precedente)/delta_tempo; //approssimazione
    errore_precedente = errore;
    tempo_precedente = tempo;
    output = (errore * Kp) + (integralSum * Ki) + (derivata * Kd);
    return output;
  }
  private double PID_w(double riferimento, double lettura){
    errore = riferimento - lettura;
    tempo = cronometro.seconds();
    delta_tempo = tempo - tempo_precedente;
    if (delta_tempo <= 0) {delta_tempo = 0.001;}
    integralSum += errore * delta_tempo;
    derivata = (errore - errore_precedente)/delta_tempo; //approssimazione
    errore_precedente = errore;
    tempo_precedente = tempo;
    output = (errore * Kp) + (integralSum * Ki) + (derivata * Kd);
    return output;
  }
  
  private File fileLog;
  private PrintWriter scrittoreFile;
  
  private void raccolta_dati(double alpha, double beta) {
    String riga = String.format(java.util.Locale.US, "%.4f  %.4f  %.4f  %.4f  %.4f", cronometro.seconds(), alpha, beta, errore, output);
    logBuffer.add(riga);
  }
  private void ScriviBufferSuFile(String nomeFile, ArrayList<String> buffer) {
    try {
      File directory = new File("/sdcard/FIRST/data");
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
