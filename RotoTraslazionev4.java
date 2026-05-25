package org.firstinspires.ftc.kikko;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp(name = "RotoTraslazionev3")
public class RotoTraslazionev3 extends LinearOpMode {

  private DcMotor sud;
  private DcMotor east;
  private DcMotor ovest;
  private IMU imu;
  
  double G = Math.toRadians(0);           //angolo di inclinazione del Motore Sud rispetto all'asse X               (0)
  double cosG = Math.cos(G);              //coseno dell'angolo di inclinazione del Motore Sud rispetto all'asse X   (1)
  double sinG = Math.sin(G);              //seno dell'angolo di inclinazione del Motore Sud rispetto all'asse X     (0)
  
  double A = Math.toRadians(120);         //angolo di inclinazione del Motore East rispetto all'asse X              (120)
  double cosA = Math.cos(A);              //coseno dell'angolo di inclinazione del Motore East rispetto all'asse X  (-1/2)
  double sinA = Math.sin(A);              //seno dell'angolo di inclinazione del Motore East rispetto all'asse X    (√3/2)
  
  double B = Math.toRadians(-120);        //angolo di inclinazione del Motore Ovest rispetto all'asse X             (-120)
  double cosB = Math.cos(B);              //coseno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X (-1/2)
  double sinB = Math.sin(B);              //seno dell'angolo di inclinazione del Motore Ovest rispetto all'asse X   (-√3/2)
  
  @Override
  public void runOpMode() {
    // Definiamo il giroscopio
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    imu = hardwareMap.get(IMU.class, "imu");
    // Definiamo l'orientazione del controller hub
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    
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
    
    waitForStart();
    if (opModeIsActive()) {
      boolean verboso = true;                 //scegli quanto far parlare il programma (true tanto; false poco)
      
      double PS, PE, PO;                      //potenza del Motore Sud, East, Ovest
      double[] P = new double[3];             //array potenze
      
      double Xl, Yl;                          //Valore X, Y Stick Sinistra
      double Xr, Yr;                          //Valore X, Y Stick Destra
      
      double p_i = 0.5;                       //da 0->1 fattore attenuazine della potenza dei motori
      double p = p_i;
      
      double theta;                           //angolo di inclinazione del robot rispetto all'asse z (giroscopio)
      double phi;                             //angolo di inclinazione dello Stick Destra
      
      double[] coord_robot = new double[3];
      
      int state = 1;                          //Tipo di moto
      String moto_attuale = "default";
      
      while (opModeIsActive()) {
        telemetry.update();
        orientation = imu.getRobotYawPitchRollAngles();
        if (gamepad1.right_stick_button) {imu.resetYaw();}                                //R3 reset giroscopio
        if (gamepad1.rightBumperWasPressed()) {state += 1; if (state > 2){state = 2;}}    //R1 aumenta stato
        if (gamepad1.leftBumperWasPressed()) {state += -1; if (state < 0){state = 0;}}    //L1 diminuisci stato
        if (gamepad1.ps) {state = 0;}                                                     //PS torna a stato default
        if (gamepad1.cross) {sud.setPower(0.5);}                                          //X accendi motore sud
        if (gamepad1.circle) {east.setPower(0.5);}                                        //O accendi motore east
        if (gamepad1.square) {ovest.setPower(0.5);}                                       //[] accendi motore ovest
        if (gamepad1.dpadUpWasPressed()) {p += 0.1; if (p > 1){p = 1;}}                   //Freccia su aumenta potenza
        if (gamepad1.dpadDownWasPressed()) {p += -0.1; if (p < 0.2){p = 0.2;}}            //Freccia giù diminuisci potenza
        if (gamepad1.start) {p = p_i;}                                                    //START reset potenza
        
        Xl =  (gamepad1.left_stick_x);
        Yl =  -(gamepad1.left_stick_y);
        Xr =  -(gamepad1.right_stick_y);
        Yr =  -(gamepad1.right_stick_x);
        phi = find_phi(Xr, Yr);
        //phi = Math.atan2(Yr, Xr);
        theta = orientation.getYaw(AngleUnit.RADIANS);
        
        switch (state){
          case 0:
            moto_attuale = "Rototralsazione";
            P = rototraslazione(Xl, Yl, phi, theta);
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
        
        sud.setPower(p*PS);
        east.setPower(p*PE);
        ovest.setPower(p*PO);
        
        if (verboso) {
          telemetry.addData("moto: ", moto_attuale);
          telemetry.addData("p", p);
          telemetry.addData("xr", Xr);
          telemetry.addData("yr", Yr);
          telemetry.addData("xl", Xl);
          telemetry.addData("yl", Yl);
          telemetry.addData("PS", PS);
          telemetry.addData("PE", PE);
          telemetry.addData("PO", PO);
          telemetry.addData("theta", Math.toDegrees(theta));
          telemetry.addData("phi", Math.toDegrees(phi));
          telemetry.addData("phi - theta", Math.toDegrees(phi - theta));
        }
      }
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
    if (max > 1) {for (int i = 0; i < 3; i++) {vettore[i] /= max;}}
    return vettore;
  }
  
  private double[] traslazione(double x, double y) {
    double[] potenze_traslazionali = new double[3];
    potenze_traslazionali = cambio_coordinate(x, y, 0);
    double norma = 0;
    for(int i = 0; i < 3; i++) {norma += Math.pow(potenze_traslazionali[i], 2);}
    norma = Math.sqrt(norma);
    if(norma !=0){for(int i = 0; i < 3; i++) {potenze_traslazionali[i] /= norma;}}
    return potenze_traslazionali;
  }
  private double[] rotazione_angolata(double alpha, double beta) {//alpha richiesto, beta letto
    double differenza = alpha-beta;
    double[] potenze_rotazionali = new double[3]; 
    //if (Math.abs(differenza) > (0.1 * Math.PI/180)) {
      for (int i = 0; i < 3; i++) {potenze_rotazionali[i] = Math.sin(differenza);}
    //}
    return potenze_rotazionali;
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
    for (int i = 0; i < 3; i++) {potenze_rotazionali[i] = Math.sin(differenza);}
    return potenze_rotazionali;
  }
  private double[] rototraslazione(double x, double y, double alpha, double beta) {
    double[] traslazionali = new double[3];
    double[] rotazionali = new double[3];
    double[] potenze_rototraslazionali = new double[3];
    traslazionali = traslazione(x, y);
    rotazionali = rotazione(PID(alpha, beta));
    for (int i = 0; i < 3; i++) {potenze_rototraslazionali[i] = traslazionali[i] + rotazionali[i];}
    return potenze_rototraslazionali;
  }
  //ElapsedTime tempotot = new ElapsedTime();
  ElapsedTime cronometro = new ElapsedTime();
  double errore = 0;
  double errore_precedente = 0;
  double tempo = 0;
  double tempo_precedente = 0;
  double delta_tempo = 0;
  double integralSum = 0;
  double derivata = 0;
  double Kp = 1;
  double Ki = 0;
  double Kd = 0;

  private double PID(double riferimento, double lettura){
    errore = riferimento - lettura;
    tempo = cronometro.seconds();
    delta_tempo = tempo - tempo_precedente;
    integralSum += errore * delta_tempo;
    derivata = (errore - errore_precedente)/delta_tempo; //approssimazione
    errore_precedente = errore;
    tempo_precedente = tempo;

    return (errore * Kp) + (integralSum * Ki) + (derivata * Kd);
  }

}
