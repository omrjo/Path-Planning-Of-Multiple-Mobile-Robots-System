using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Microsoft.DirectX.DirectInput;
using Microsoft.DirectX;
using System.Threading;
using System.Diagnostics;

namespace X80Demo
{

    public partial class Form1 : Form
    {
        //here is some import parameter depending on what kind of robot you are using
        //here is for X80 robot
        private const double WheelDis = 0.26;
        private const double WheelR = 0.085;   //0.0825
        private const Int16 CircleCnt = 1200;       //one circle encoder value
        private int MotorDemoStep = 0;
        private int ServoDemoStep = 0;


        private Device applicationDevice = null;
        public static int numPOVs = 0;


        private int BrakeLevel = 10000;
        private int GasLevel = 0;
        private int SteerLevel = 5000;
        private int BackwardLevel = 5000;
        private int GearLevel = 1;


        private double leftwheel_PWM = 0;
        private double rightwheel_PWM = 0;

        private const double stallY = 0.01;
        private const int FricL = 6000;
        private const int FricR = 6000;



        //for listen
        private Boolean blnListen = false;
        private WaveLib.WaveOutPlayer m_Player;
        private WaveLib.FifoStream m_Fifo = new WaveLib.FifoStream();

        private byte[] m_PlayBuffer;
        private byte[] m_RecBuffer;


        public Form1()
        {
            InitializeComponent();

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            myRobot.connectRobot("drrobot1");
            tmrCamera.Enabled = true;
            if (InitDirectInput())
                tmrJoyStickPoll.Start();

        }

        private bool InitDirectInput()
        {
            // Enumerate joysticks in the system.
            foreach (DeviceInstance instance in Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly))
            {
                // Create the device.  Just pick the first one
                applicationDevice = new Device(instance.InstanceGuid);
                break;
            }

            if (null == applicationDevice)
            {
                MessageBox.Show("Unable to create a joystick device.", "No joystick found");
                return false;
            }

            // Set the data format to the c_dfDIJoystick pre-defined format.
            applicationDevice.SetDataFormat(DeviceDataFormat.Joystick);
            // Set the cooperative level for the device.
            applicationDevice.SetCooperativeLevel(this, CooperativeLevelFlags.Exclusive | CooperativeLevelFlags.Foreground);
            // Enumerate all the objects on the device.
            foreach (DeviceObjectInstance d in applicationDevice.Objects)
            {
                // For axes that are returned, set the DIPROP_RANGE property for the
                // enumerated axis in order to scale min/max values.

                if ((0 != (d.ObjectId & (int)DeviceObjectTypeFlags.Axis)))
                {
                    // Set the range for the axis.
                    applicationDevice.Properties.SetRange(ParameterHow.ById, d.ObjectId, new InputRange(0, +10000));
                }
                // Update the controls to reflect what
                // objects the device supports.
                //  UpdateControls(d);
            }
            return true;
        }



        private void RobotDrive()
        {


            Steeringwheel2PWM();



            SendWheelDataToRobot();
        }
        private void SendWheelDataToRobot()
        {
            short LeftWheelValue = 16383;
            short RightWheelValue = 16383;

            LeftWheelValue = (short)(leftwheel_PWM * 16383 + 16383);
            RightWheelValue = (short)(rightwheel_PWM * 16383 + 16384);

            if (LeftWheelValue > 32766)
                LeftWheelValue = 32766;

            if (LeftWheelValue < 0)
                LeftWheelValue = 0;

            if (RightWheelValue > 32766)
                RightWheelValue = 32766;

            if (RightWheelValue < 0)
                RightWheelValue = 0;


            myRobot.DcMotorPwmNonTimeCtr(0, LeftWheelValue);
            myRobot.DcMotorPwmNonTimeCtr(1, RightWheelValue);
        }


        private void Steeringwheel2PWM()
        {

            double Y = 0;
            double z = 0;
            int PWMValue = 0;

            double PowerR = 0;
            int maxFric = 0;

            if (BackwardLevel < 5500) //forward drive
                Y = ((double)(10000 - GasLevel)) / 10000.0;
            else
                Y = -((double)(10000 - GasLevel)) / 10000.0;



            z = ((double)(SteerLevel - 5000)) / 5000.0 / (1 + (double)GearLevel * 0.2);

            PowerR = (double)GearLevel * 0.3333;

            maxFric = FricL;


            PWMValue = (int)(Y * (double)(16383 - maxFric));

            if ((Math.Abs(Y) > stallY) && (BrakeLevel > 7000) && (GearLevel != 0))
                if (Y >= 0)
                { //forward
                    if (z <= 0)
                    {
                        rightwheel_PWM = 16383 + FricR + (int)((double)PWMValue * PowerR);
                        leftwheel_PWM = 16383 + Math.Sign(0.5 + 0.5 * (1 + 2 * z)) * (double)FricL + PWMValue * PowerR * (0.5 + 0.5 * (1 + 2 * z));
                    }

                    else
                    {
                        rightwheel_PWM = 16383 + Math.Sign(0.5 + 0.5 * (1 - 2 * z)) * FricR + PWMValue * PowerR * (0.5 + 0.5 * (1 - 2 * z));
                        leftwheel_PWM = 16383 + FricL + PWMValue * PowerR;
                    }

                }
                else
                { //backard
                    if (z <= 0)
                    {
                        leftwheel_PWM = 16383 - Math.Sign(0.5 + 0.5 * (1 + 2 * z)) * FricL + PWMValue * PowerR * (0.5 + 0.5 * (1 + 2 * z));
                        rightwheel_PWM = 16383 - FricR + PWMValue * PowerR;
                    }
                    else
                    {
                        leftwheel_PWM = 16383 - FricL + PWMValue * PowerR;
                        rightwheel_PWM = 16383 - Math.Sign(0.5 + 0.5 * (1 - 2 * z)) * FricR + PWMValue * PowerR * (0.5 + 0.5 * (1 - 2 * z));
                    }

                }

            else
            {
                rightwheel_PWM = 16383;
                leftwheel_PWM = 16383;
            }


            leftwheel_PWM = -(leftwheel_PWM - 16383) / 16383;
            rightwheel_PWM = (rightwheel_PWM - 16383) / 16383;





        }


        private void myRobot_StandardSensorEvent(object sender, EventArgs e)
        {
            lblUSDis1.Text = ((double)(myRobot.GetSensorSonar1()) / 100).ToString() + "m";
            lblUSDis2.Text = ((double)(myRobot.GetSensorSonar2()) / 100).ToString() + "m";
            lblUSDis3.Text = ((double)(myRobot.GetSensorSonar3()) / 100).ToString() + "m";
            lblUSDis4.Text = ((double)(myRobot.GetSensorSonar4()) / 100).ToString() + "m";
            lblUSDis5.Text = ((double)(myRobot.GetSensorSonar5()) / 100).ToString() + "m";
            lblUSDis6.Text = ((double)(myRobot.GetSensorSonar6()) / 100).ToString() + "m";

            lblIRDis1.Text = (AD2Dis(myRobot.GetSensorIRRange())).ToString("0.00") + "m";
            lblBoardVol.Text = ((double)(myRobot.GetSensorBatteryAD1()) / 4095.0 * 9).ToString("0.00") + "V";
            lblMotorVol.Text = ((double)(myRobot.GetSensorBatteryAD2()) / 4095.0 * 24).ToString("0.00") + "V";
            lblLeftAlarm.Text = (myRobot.GetSensorHumanAlarm1()).ToString();
            lblLeftMotion.Text = (myRobot.GetSensorHumanMotion1()).ToString();
            lblRightAlarm.Text = (myRobot.GetSensorHumanAlarm2()).ToString();
            lblRightMotion.Text = (myRobot.GetSensorHumanMotion2()).ToString();
        }

        private void btnForward_Click(object sender, EventArgs e)
        {
            //short speed = (short)trackBar1.Value;
            myRobot.DcMotorVelocityNonTimeCtrAll(470, -470, -32767, -32767, -32767, -32767);
            Thread.Sleep(550);

            myRobot.SuspendDcMotor(0);
            myRobot.SuspendDcMotor(1);
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            myRobot.DisableDcMotor(0);
            myRobot.DisableDcMotor(1);
        }

        private void tmrCamera_Tick(object sender, EventArgs e)
        {
            myRobot.TakePhoto();
        }

        private void btnLeft_Click(object sender, EventArgs e)
        {
            short speed = (short)trackBar1.Value;
            myRobot.DcMotorVelocityNonTimeCtrAll(speed, speed, -32767, -32767, -32767, -32767);
        }

        private void btnBack_Click(object sender, EventArgs e)
        {
            short speed = (short)trackBar1.Value;
            myRobot.DcMotorVelocityNonTimeCtrAll(speed, (short)-speed, -32767, -32767, -32767, -32767);
        }

        private void btnRight_Click(object sender, EventArgs e)
        {
            short speed = (short)trackBar1.Value;
            myRobot.DcMotorVelocityNonTimeCtrAll((short)-speed, (short)-speed, -32767, -32767, -32767, -32767);
        }

        private void myRobot_MotorSensorEvent(object sender, EventArgs e)
        {
            lblLeftPos.Text = (myRobot.GetEncoderPulse1()).ToString();
            lblRightPos.Text = (myRobot.GetEncoderPulse2()).ToString();

            lblLeftSpeed.Text = (myRobot.GetEncoderSpeed1()).ToString();
            lblRightSpeed.Text = (myRobot.GetEncoderSpeed2()).ToString();
            lblLeftCurrent.Text = ((double)myRobot.GetMotorCurrent1() / 728.0).ToString("0.00") + "A";
            lblRightCurrent.Text = ((double)myRobot.GetMotorCurrent2() / 728.0).ToString("0.00") + "A";

        }

        private void myRobot_CustomSensorEvent(object sender, EventArgs e)
        {
            lblIRDis2.Text = (AD2Dis(myRobot.GetCustomAD3())).ToString("0.00") + "m";
            lblIRDis3.Text = (AD2Dis(myRobot.GetCustomAD4())).ToString("0.00") + "m";
            lblIRDis4.Text = (AD2Dis(myRobot.GetCustomAD5())).ToString("0.00") + "m";
            lblIRDis5.Text = (AD2Dis(myRobot.GetCustomAD6())).ToString("0.00") + "m";
            lblIRDis6.Text = (AD2Dis(myRobot.GetCustomAD7())).ToString("0.00") + "m";
            lblIRDis7.Text = (AD2Dis(myRobot.GetCustomAD8())).ToString("0.00") + "m";

        }

        private void btnForward1M_Click(object sender, EventArgs e)
        {
            RunDis(1.0);

        }

        private void btnTurn90_Click(object sender, EventArgs e)
        {
            Turn(90);

        }

        private void btnDemo_Click(object sender, EventArgs e)
        {
            MotorDemoStep = 0;
            RunDis(1.0); //forward 1m
            tmrMotorDemo.Enabled = true;

        }
        private void Turn(double degreee)
        {
            double Dis = WheelDis / 2 * (degreee / 180.0 * Math.PI);
            int diffEncoder = (int)(Dis / (2 * Math.PI * WheelR) * CircleCnt);

            int LeftCmd = int.Parse(lblLeftPos.Text) + diffEncoder;
            if (LeftCmd < 0)
            {
                LeftCmd = 32767 + LeftCmd;
            }
            else if (LeftCmd > 32767)
            {
                LeftCmd = LeftCmd - 32767;
            }

            int RightCmd = int.Parse(lblRightPos.Text) + diffEncoder;
            if (RightCmd < 0)
            {
                RightCmd = 32767 + RightCmd;
            }
            else if (RightCmd > 32767)
            {
                RightCmd = RightCmd - 32767;
            }
            myRobot.SetDcMotorPositionControlPID(0, 1000, 5, 10000);
            myRobot.SetDcMotorPositionControlPID(1, 1000, 5, 10000);
            myRobot.DcMotorPositionTimeCtrAll((short)LeftCmd, (short)RightCmd, -32767, -32767, -32767, -32767, 4000);
        }



        private void RunDis(double Dis)
        {
            Dis = 1.0;        //1m
            int diffEncoder = (int)(Dis / (2 * Math.PI * WheelR) * CircleCnt);

            int LeftCmd = int.Parse(lblLeftPos.Text) - diffEncoder;
            if (LeftCmd < 0)
            {
                LeftCmd = 32767 + LeftCmd;
            }
            else if (LeftCmd > 32767)
            {
                LeftCmd = LeftCmd - 32767;
            }

            int RightCmd = int.Parse(lblRightPos.Text) + diffEncoder;
            if (RightCmd < 0)
            {
                RightCmd = 32767 + RightCmd;
            }
            else if (RightCmd > 32767)
            {
                RightCmd = RightCmd - 32767;
            }

            myRobot.DcMotorPositionTimeCtrAll((short)LeftCmd, (short)RightCmd, -32767, -32767, -32767, -32767, 10000);

        }

        private void tmrMotorDemo_Tick(object sender, EventArgs e)
        {
            MotorDemoStep++;
            if (MotorDemoStep == 1)
                Turn(90);
            else if (MotorDemoStep == 2)
                RunDis(1.0);
            else if (MotorDemoStep == 3)
                Turn(90);
            else if (MotorDemoStep == 4)
                RunDis(1.0);
            else if (MotorDemoStep == 5)
                Turn(90);
            else if (MotorDemoStep == 6)
                RunDis(1.0);
            else if (MotorDemoStep == 7)
                Turn(90);
            else if (MotorDemoStep == 8)
            {
                //stop here
                tmrMotorDemo.Enabled = false;
                MotorDemoStep = -1;
            }

        }

        private void btnServoDemo_Click(object sender, EventArgs e)
        {
            ServoDemoStep = 0;
            //3500, 3200, yo can modify the value based on your robot
            myRobot.ServoTimeCtrAll(3500, 3200, -32767, -32767, -32767, -32767, 1500);
            tmrServoDemo.Enabled = true;

        }

        private void tmrServoDemo_Tick(object sender, EventArgs e)
        {
            ServoDemoStep++;
            if (ServoDemoStep == 1)
                myRobot.ServoTimeCtrAll(4200, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 2)
                myRobot.ServoTimeCtrAll(2400, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 3)
                myRobot.ServoTimeCtrAll(4200, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 4)
                myRobot.ServoTimeCtrAll(2400, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 5)
                myRobot.ServoTimeCtrAll(3500, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 6)
                myRobot.ServoTimeCtrAll(3500, 4400, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 7)
                myRobot.ServoTimeCtrAll(3500, 2400, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 8)
                myRobot.ServoTimeCtrAll(3500, 4400, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 9)
                myRobot.ServoTimeCtrAll(3500, 2400, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 10)
                myRobot.ServoTimeCtrAll(3500, 3200, -32767, -32767, -32767, -32767, 1500);
            else if (ServoDemoStep == 1)
            {
                ServoDemoStep = -1;
                tmrServoDemo.Enabled = false;
            }

        }


        private double AD2Dis(Int16 IRValue)
        {
            double temp = 0;
            double IRAD2Distance = 0;

            temp = 21.6 / ((double)IRValue * 3 / 4028 - 0.17);

            // IR range 10-80cm
            if ((temp > 80) || (temp < 0))
                IRAD2Distance = 0.81;
            else if ((temp < 10) && (temp > 0))
                IRAD2Distance = 0.09;
            else
                IRAD2Distance = temp / 100;
            return IRAD2Distance;
        }



        private void trackBar2_Scroll(object sender, EventArgs e)
        {
            Int16 cmdValue = (short)trackBar2.Value;
            myRobot.ServoTimeCtr(1, (short)cmdValue, 1000);

        }

        private void trackBar3_Scroll(object sender, EventArgs e)
        {
            Int16 cmdValue = (short)trackBar3.Value;
            myRobot.ServoTimeCtr(0, (short)cmdValue, 1000);

        }

        private void btnSaveImage_Click(object sender, EventArgs e)
        {
            myRobot.SavePhotoAsBMP(Environment.CurrentDirectory + "\\photo.bmp");

        }

        private void btnPlay_Click(object sender, EventArgs e)
        {
            if (btnPlay.Text == "PlayMusic")
            {

                openFileDialog1.InitialDirectory = ".\\";
                openFileDialog1.Filter = "wave Files (*.wav)|*.wav";
                openFileDialog1.Title = "Type File";
                //openFileDialog1.ShowDialog();

                if (openFileDialog1.ShowDialog() != DialogResult.OK)
                    return;
                else
                {
                    myRobot.PlayAudioFile(openFileDialog1.FileName);
                    btnPlay.Text = "StopMusic";

                }

            }
            else
            {
                myRobot.StopAudioPlay();
                btnPlay.Text = "PlayMusic";
            }
        }

        private void btnListening_Click(object sender, EventArgs e)
        {
            if (btnListening.Text == "Start Listen")
            {
                btnListening.Text = "Stop Listen";
                blnListen = true;
                //prepare buffer
                Stop();
                try
                {
                    WaveLib.WaveFormat fmt = new WaveLib.WaveFormat(8000, 16, 1);
                    m_Player = new WaveLib.WaveOutPlayer(-1, fmt, 8192, 3, new WaveLib.BufferFillEventHandler(Filler));
                    myRobot.StartRecord(1);     //about 256ms, 4096 data
                }
                catch
                {
                    Stop();
                    //throw;
                }

            }
            else
            {
                btnListening.Text = "Start Listen";
                blnListen = false;
                myRobot.StopRecord();
                Stop();
            }

        }

        private void Filler(IntPtr data, int size)
        {
            if (m_PlayBuffer == null || m_PlayBuffer.Length < size)
                m_PlayBuffer = new byte[size];
            if (m_Fifo.Length >= size)
                m_Fifo.Read(m_PlayBuffer, 0, size);
            else
                for (int i = 0; i < m_PlayBuffer.Length; i++)
                    m_PlayBuffer[i] = 0;
            System.Runtime.InteropServices.Marshal.Copy(m_PlayBuffer, 0, data, size);
        }

        private void Stop()
        {
            if (m_Player != null)
                try
                {
                    m_Player.Dispose();
                }
                finally
                {
                    m_Player = null;
                }

            m_Fifo.Flush(); // clear all pending data
        }

        private void myRobot_VoiceSegmentEvent(object sender, EventArgs e)
        {
            if (blnListen)
            {
                //m_Fifo.Flush();
                int size = myRobot.GetVoiceSegLength();
                IntPtr data = (IntPtr)myRobot.GetVoiceSegment();
                if (m_RecBuffer == null || m_RecBuffer.Length < size)
                    m_RecBuffer = new byte[size];
                System.Runtime.InteropServices.Marshal.Copy(data, m_RecBuffer, 0, size);
                m_Fifo.Write(m_RecBuffer, 0, m_RecBuffer.Length);
            }
        }

        private void Form1_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            myRobot.StopRecord();
            tmrCamera.Enabled = false;
            tmrJoyStickPoll.Enabled = false;
            // Unacquire all DirectInput objects.
            if (null != applicationDevice)
                applicationDevice.Unacquire();

            Stop();
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start("http://www.drrobot.com");
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            Close();
        }




        bool ispositive(int a)
        {
            if (a >= 0) return true;
            else return false;

        }

        int compelement(int a)
        {
            bool b = true;
            if (!ispositive(a))
            {
                a = a * (-1);
                b = false;
            }

            a = 8 - a;

            if (b)
            {
                a = a * (-1);
            }

            return a;

        }

        void edit(short turn, short tt)
        {
            short turnspeed = turn, turntime = tt;
            int k = nav.direction - nav.nxtdirection;


            if (k == 0)
            {
                //no need to do any action 
            }
            else if (k < 4 && k > -4)
            {
                if (ispositive(k))
                //turn left k times
                {
                    myRobot.DcMotorVelocityNonTimeCtrAll((short)-turnspeed, (short)-turnspeed, -32767, -32767, -32767, -32767);
                    Thread.Sleep(turntime * k);

                }
                else
                {
                    //turn right k times
                    k = k * -1;
                    myRobot.DcMotorVelocityNonTimeCtrAll(turnspeed, turnspeed, -32767, -32767, -32767, -32767);
                    Thread.Sleep(turntime * k);
                }
            }
            else if (k == 4 || k == -4)
            {
                nav.backword = true;

            }
            else
            {

                k = compelement(k);

                if (ispositive(k))
                //turn left k times
                {
                    myRobot.DcMotorVelocityNonTimeCtrAll((short)-turnspeed, (short)-turnspeed, -32767, -32767, -32767, -32767);
                    Thread.Sleep(turntime * k);
                }
                else
                {
                    //turn right k times
                    k = k * -1;
                    myRobot.DcMotorVelocityNonTimeCtrAll(turnspeed, turnspeed, -32767, -32767, -32767, -32767);
                    Thread.Sleep(turntime * k);
                }
            }
        }

        public void StopToDetectTheRobotInNextPoint()
        {

            myRobot.SuspendDcMotor(0);
            myRobot.SuspendDcMotor(1);
            Thread.Sleep(5000);
            nav.middlesensor = myRobot.GetSensorSonar2();

        }

        public void MoveToNextPoint(short d, short s, short t, bool diag)
        {
            SetSensorsToMax();
            Stopwatch timer = new Stopwatch();
            bool diagonal = diag, detect = false;
            short direction = d, speed = s, time = t, counter = 0, TotalTime = 0;
            bool Done = false;


            while (!Done)
            {
                if (ispositive(speed))
                { //if speed is positive that means it is forward
                    if (diagonal)
                    {
                        nav.firstl = 45;
                        nav.firstr = 45;
                        nav.firstm = 55;
                        myRobot.DcMotorVelocityNonTimeCtrAll(speed, (short)-speed, -32767, -32767, -32767, -32767);
                        timer.Start();
                        while (counter < 5)
                        {
                            Thread.Sleep(time / 5);
                            GetSensorsData();
                            if (nav.middlesensor <= nav.firstm - counter * 5 || nav.leftsensor <= nav.firstl - counter * 2 || nav.rightsensor <= nav.firstr - counter * 2)
                            {
                                myRobot.SuspendDcMotor(0);
                                myRobot.SuspendDcMotor(1);
                                TotalTime = (short)timer.ElapsedMilliseconds;
                                timer.Stop();
                                counter++;
                                detect = true;
                                nav.direction = direction;
                                break;
                            }
                        }
                        myRobot.SuspendDcMotor(0);
                        myRobot.SuspendDcMotor(1);
                    }
                    else//not diagonal
                    {
                        nav.firstl = 17;
                        nav.firstr = 17;
                        nav.firstm = 30;
                        myRobot.DcMotorVelocityNonTimeCtrAll(speed, (short)-speed, -32767, -32767, -32767, -32767);
                        timer.Start();
                        while (counter < 5)
                        {
                            Thread.Sleep(time / 5);
                            GetSensorsData();
                            if (nav.middlesensor <= nav.firstm - counter * 5 || nav.leftsensor <= nav.firstl - counter * 2 || nav.rightsensor <= nav.firstr - counter * 2)
                            {
                                myRobot.SuspendDcMotor(0);
                                myRobot.SuspendDcMotor(1);
                                TotalTime = (short)timer.ElapsedMilliseconds;
                                timer.Stop();
                                counter++;
                                detect = true;
                                nav.direction = direction;
                                break;
                            }
                        }
                        myRobot.SuspendDcMotor(0);
                        myRobot.SuspendDcMotor(1);
                    }
                }//nigative which means it is backword
                else
                {
                    if (diagonal)
                    {
                        nav.firstl = 45;
                        nav.firstr = 45;
                        nav.firstm = 55;
                        myRobot.DcMotorVelocityNonTimeCtrAll(speed, (short)-speed, -32767, -32767, -32767, -32767);
                        timer.Start();
                        while (counter < 5)
                        {
                            Thread.Sleep(time / 5);
                            GetSensorsData();
                            if (nav.BackMiddleSensor <= nav.firstm - counter * 5 || nav.BackLeftSensor<= nav.firstl - counter * 2 || nav.BackRightSensor <= nav.firstr - counter * 2)
                            {
                                myRobot.SuspendDcMotor(0);
                                myRobot.SuspendDcMotor(1);
                                TotalTime = (short)timer.ElapsedMilliseconds;
                                timer.Stop();
                                counter++;
                                detect = true;
                                nav.direction = direction;
                                break;
                            }
                        }
                        myRobot.SuspendDcMotor(0);
                        myRobot.SuspendDcMotor(1);
                    }
                    else//not diagonal
                    { }
                }




                if (detect)
                {
                    Random rnd = new Random();
                    short priority = (short)rnd.Next(1, 3);
                    bool done = false;
                    detect = false;


                    while (!done)
                    {
                        if (priority == 1)//i will go
                        {
                            int time1 = (int)time, totaltime = (int)TotalTime, TimeLeft = time1 - totaltime;
                            Thread.Sleep(2000);
                            GetSensorsData();

                            if (ispositive(speed))
                            {
                                if (nav.middlesensor > nav.firstm - counter - 1 * 5 && nav.leftsensor > nav.firstl - counter - 1 * 2 && nav.rightsensor > nav.firstr - counter - 1 * 2)
                                {
                                    break;
                                }
                                else
                                {
                                    priority = (short)rnd.Next(1, 3);
                                }
                            }
                            else {
                                if (nav.BackMiddleSensor > nav.firstm - counter - 1 * 5 && nav.BackLeftSensor > nav.firstl - counter - 1 * 2 && nav.BackRightSensor > nav.firstr - counter - 1 * 2)
                                {
                                    break;
                                }
                                else
                                {
                                    priority = (short)rnd.Next(1, 3);
                                }
                            
                            
                            }


                        }
                        else
                        {//priority = 2 so i will do a step back 
                            short tmpspeed = (short)-speed;
                            myRobot.DcMotorVelocityNonTimeCtrAll(tmpspeed, (short)-tmpspeed, -32767, -32767, -32767, -32767);
                            Thread.Sleep(TotalTime);
                            myRobot.SuspendDcMotor(0);
                            myRobot.SuspendDcMotor(1);
                            nav.obstacle = true;
                            done = true;
                            Done = true;
                        }

                    }

                }
            }
        }


        public void turn(short ts, short tt)
        {
            short turnspeed = ts, turntime = tt;
            myRobot.DcMotorVelocityNonTimeCtrAll(turnspeed, turnspeed, -32767, -32767, -32767, -32767);
            Thread.Sleep(turntime);
            myRobot.SuspendDcMotor(0);
            myRobot.SuspendDcMotor(1);
        }

        public void GetSensorsData() {

            nav.middlesensor = myRobot.GetSensorSonar2();
            nav.leftsensor = myRobot.GetSensorSonar1();
            nav.rightsensor = myRobot.GetSensorSonar3();
            nav.BackMiddleSensor = myRobot.GetSensorSonar5();
            nav.BackRightSensor = myRobot.GetSensorSonar4();
            nav.BackLeftSensor = myRobot.GetSensorSonar6();
        
        }

        public void SetSensorsToMax()
        {
            nav.middlesensor = short.MaxValue;
            nav.leftsensor = short.MaxValue;
            nav.rightsensor = short.MaxValue;
            nav.BackMiddleSensor = short.MaxValue;
            nav.BackRightSensor = short.MaxValue;
            nav.BackLeftSensor = short.MaxValue;
        }
        
        
        
        public void move(Point cur, Point nxt, short s, short t, short dt, short nt, short tt)
        {
            short speed = s, turnspeed = t, diagonaltime = dt, normaltime = nt, turntime = tt;
            int width = map.GetLength(0), height = map.GetLength(1);
            Point left = new Point(), right = new Point();
            left.X = cur.X - 1;
            left.Y = cur.Y - 1;
            right.X = cur.X + 1;
            right.Y = cur.Y + 1;


            //A
            if (nxt.Y == cur.Y + 1)
            {

                //A1
                if (nxt.X == cur.X)
                {
                    nav.nxtdirection = 0;
                    //go stright 40cm
                    //if the dicrtion is not to the north edit it and direct it to the next location
                    if (nav.direction != 0)
                    {
                        edit(turnspeed, turntime);
                    }
                    //else if direction is to the north go direct to the next location40cm
                    if (nav.backword)
                    {//if the next node is in the back of your direction just go back without rotating for saving time
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.BackMiddleSensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.BackMiddleSensor <= 40)
                        {
                            nav.direction = 4;
                            nav.obstacle = true;
                            nav.backword = false;
                        }
                        else
                        {//else if the location is empty move to it 
                            MoveToNextPoint(4, (short)-speed, normaltime,false);
                            nav.backword = false;
                        }
                    }
                    else
                    {//if the next node is not in the back of your direction just go forward 
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.middlesensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.middlesensor <= 40)
                        {
                            nav.direction = 0;
                            nav.obstacle = true;
                        }
                        else
                        {//else if the location is empty move to it 
                            MoveToNextPoint(0, speed, normaltime,false);
                        }
                    }
                }//A2
                else if (nxt.X == cur.X + 1)
                {//turn right 45degree then go stright 56.56cm becuase it is diagonal edge

                    nav.nxtdirection = 1;
                    
                    //if the dicrtion is not to the north edit it and direct it to the next location
                    if (nav.direction != 0)
                    {
                        edit(turnspeed, turntime);
                    } 
                    else
                    {//else if direction is to the north turn right 45degree
                        turn(turnspeed, turntime);
                    }
                    
                    if (nav.backword)
                    {//if the next node is in the back of your direction just go back without rotating for saving time
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.BackMiddleSensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.BackMiddleSensor <= 40)
                        {
                        nav.direction = 5;
                        nav.obstacle = true;
                        nav.backword = false;
                        }
                        else if (nav.BackLeftSensor <= 35)
                        {//if there is a robot in lift replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 5;
                            nav.obstacle = true;
                            ob.X = cur.X;
                            ob.Y = cur.Y + 1;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else if (nav.BackRightSensor <= 35)
                        {//if there is a robot in right replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 5;
                            nav.obstacle = true;
                            ob.X = cur.X + 1;
                            ob.Y = cur.Y;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(5, (short)-speed, diagonaltime,true);
                            nav.backword = false;
                        }
                    }
                    else
                    {//if the next node is not in the back of your direction just go forward 
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.middlesensor <= 40)
                        {//if you find an obstacle it would be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.middlesensor <= 40)
                        {
                            nav.direction = 1;
                            nav.obstacle = true;
                        }
                        else if (nav.leftsensor <= 35) 
                        {//if there is a robot in lift replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 1;
                            nav.obstacle = true;
                            ob.X = cur.X;
                            ob.Y = cur.Y + 1;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else if (nav.rightsensor <= 35)
                        {//if there is a robot in right replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 1;
                            nav.obstacle = true;
                            ob.X = cur.X+1;
                            ob.Y = cur.Y;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(1, speed, diagonaltime,true);
                        }
                    }
                }//A3
                else if (nxt.X == cur.X - 1)
                { //turn left 45degree then go stright 56.56cm becuase it is diagonal edge
                    nav.nxtdirection = 7;
                    
                    if (nav.direction != 0)//if the dicrtion is not to the north edit it and direct it to the next location
                    {
                        edit(turnspeed, turntime);
                    }//else if direction is to the north direct to the next location by otating lift 45degrss
                    else
                    {
                        turn((short)-turnspeed, turntime);
                    }
                    
                    if (nav.backword)
                    {//if the next node is in the back of your direction just go back without rotating for saving time
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.BackMiddleSensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.BackMiddleSensor <= 40)
                        {
                            nav.direction = 3;
                            nav.obstacle = true;
                            nav.backword = false;
                        }
                        else if (nav.BackLeftSensor <= 35)
                        {//if there is a robot in lift replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 3;
                            nav.obstacle = true;
                            ob.X = cur.X;
                            ob.Y = cur.Y + 1;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else if (nav.BackRightSensor <= 35)
                        {//if there is a robot in right replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 3;
                            nav.obstacle = true;
                            ob.X = cur.X - 1;
                            ob.Y = cur.Y;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(3, (short)-speed, diagonaltime,true);
                            nav.backword = false;
                        }
                    }
                    else
                    {//if the next node is not in the back of your direction just go forward 
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.middlesensor <= 40)
                        {//if you find an obstacle it would be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.middlesensor <= 40)
                        {
                            nav.direction = 7;
                            nav.obstacle = true;
                        }
                        else if (nav.leftsensor <= 35)
                        {//if there is a robot in lift replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 7;
                            nav.obstacle = true;
                            ob.X = cur.X-1;
                            ob.Y = cur.Y;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else if (nav.rightsensor <= 35)
                        {//if there is a robot in right replan and make it obstacle
                            //bacuse robot will crash on other robot on his way
                            Point ob = new Point();
                            nav.direction = 7;
                            nav.obstacle = true;
                            ob.X = cur.X;
                            ob.Y = cur.Y+1;
                            AddObestacle(ob);
                            nav.diagonalobstacle = true;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(7, speed, diagonaltime,true);
                        }
                    }
                }
            }//B
            else if (nxt.Y == cur.Y)
            {//B1
                if (nxt.X == cur.X + 1)
                {//turn right 90 degree then go 40cm
                    nav.nxtdirection = 2;
                    if (nav.direction != 0)
                    {//if the dicrtion is not to the north edit it and direct it to the next location
                        edit(turnspeed, turntime);
                    }//else if direction is to the north turn right 90 degree to direct the robot to the next location
                    else
                    {
                        int turntime1 = 2 * turntime;
                        turn(turnspeed, (short)turntime1);
                    }
                    if (nav.backword)
                    {//if the next node is in the back of your direction just go back without rotating for saving time
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.BackMiddleSensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.BackMiddleSensor <= 40)
                        {
                            nav.direction = 6;
                            nav.obstacle = true;
                            nav.backword = false;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(6, (short)-speed, normaltime,false);
                            nav.backword = false;
                        }
                    }
                    else
                    {//if the next node is not in the back of your direction just go forward 
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.middlesensor <= 40)
                        {//if you find an obstacle it would be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.middlesensor <= 40)
                        {
                            nav.direction = 2;
                            nav.obstacle = true;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(2, speed, normaltime,false);
                        }
                    }

                }
                else if (nxt.X == cur.X - 1)//B2
                {//turn lift 90 digree then go 40cm
                    nav.nxtdirection = 6;
                    if (nav.direction != 0)
                    {//if the dicrtion is not to the north edit it and direct it to the next location
                        edit(turnspeed, turntime);
                    }//else if direction is to the north turn lift 90 degree to direct it to the next location
                    else
                    {
                        int turntime1 = 2 * turntime;
                        turn((short)-turnspeed, (short)turntime1);
                    }

                    if (nav.backword)
                    {//if the next node is in the back of your direction just go back without rotating for saving time
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.BackMiddleSensor <= 40)
                        {//if you find and obstacle that sure will be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.BackMiddleSensor <= 40)
                        {
                            nav.direction = 2;
                            nav.obstacle = true;
                            nav.backword = false;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(2, (short)-speed, normaltime,false);
                            nav.backword = false;
                        }
                    }
                    else
                    {//if the next node is not in the back of your direction just go forward 
                        GetSensorsData();
                        //chick sensors to make sure that the naxt location is empty
                        if (nav.middlesensor <= 40)
                        {//if you find an obstacle it would be a robot so wait 5 seconds
                            StopToDetectTheRobotInNextPoint();
                        }
                        //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                        if (nav.middlesensor <= 40)
                        {
                            nav.direction = 6;
                            nav.obstacle = false;
                        }
                        else
                        {//else if the location is empty move to it
                            MoveToNextPoint(6, speed, normaltime,false);
                        }
                    }
                }//C
                else if (nxt.Y == cur.Y - 1)
                {
                    if (nxt.X == cur.X)//C1
                    {//go back 40cm
                        nav.nxtdirection = 4;
                        if (nav.direction != 0)
                        {//if the dicrtion is not to the north edit it and direct it to the next location
                            edit(turnspeed, turntime);
                        }//else if direction is to the north let it it would go back after chicking sensors
                        
                        GetSensorsData();
                        //go stright to the next point 40cm
                        if (nav.backword)
                        {//if the next node is in the back of your direction just go back without rotating for saving time
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.BackMiddleSensor <= 40)
                            {//if you find and obstacle that sure will be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.BackMiddleSensor <= 40)
                            {
                                nav.direction = 0;
                                nav.obstacle = true;
                                nav.backword = false;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(0, (short)-speed, normaltime,false);
                                nav.backword = false;
                            }
                        }
                        else
                        {//if the next node is not in the back of your direction just go forward 
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.middlesensor <= 40)
                            {//if you find an obstacle it would be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.middlesensor <= 40)
                            {
                                nav.direction = 4;
                                nav.obstacle = true;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(4, speed, normaltime,false);
                            }
                        }
                    }
                    else if (nxt.X == cur.X + 1)//C2
                    {//turn right 135 degree then go 56.56cm
                        nav.nxtdirection = 3;
                        if (nav.direction != 0)
                        {//if the dicrtion is not to the north edit it and direct it to the next location
                            edit(turnspeed, turntime);
                        }//else if direction is to the north turn right 135 degree to direct the robot to the next location
                        else
                        {
                            int turntime1 = 3 * turntime;
                            turn(turnspeed, (short)turntime1);
                        }
                        if (nav.backword)
                        {//if the next node is in the back of your direction just go back without rotating for saving time
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.BackMiddleSensor <= 40)
                            {//if you find and obstacle that sure will be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.BackMiddleSensor <= 40)
                            {
                                nav.direction = 7;
                                nav.obstacle = true;
                                nav.backword = false;
                            }
                            else if (nav.BackLeftSensor <= 35)
                            {//if there is a robot in lift replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 7;
                                nav.obstacle = true;
                                ob.X = cur.X;
                                ob.Y = cur.Y - 1;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else if (nav.BackRightSensor <= 35)
                            {//if there is a robot in right replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 7;
                                nav.obstacle = true;
                                ob.X = cur.X + 1;
                                ob.Y = cur.Y;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(7, (short)-speed, diagonaltime,true);
                                nav.backword = false;
                            }
                        }
                        else
                        {//if the next node is not in the back of your direction just go forward 
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.middlesensor <= 40)
                            {//if you find an obstacle it would be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.middlesensor <= 40)
                            {
                                nav.direction = 3;
                                nav.obstacle = true;
                            }
                            else if (nav.leftsensor <= 35)
                            {//if there is a robot in lift replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 3;
                                nav.obstacle = true;
                                ob.X = cur.X + 1;
                                ob.Y = cur.Y;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else if (nav.rightsensor <= 35)
                            {//if there is a robot in right replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 3;
                                nav.obstacle = true;
                                ob.X = cur.X;
                                ob.Y = cur.Y - 1;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(3, speed, diagonaltime,true);
                            }
                        }
                    }
                    else if (nxt.X == cur.X - 1)//C3
                    {//turn lift 135 degree then go 56.56cm
                        nav.nxtdirection = 5;
                        if (nav.direction != 0)
                        {//if the dicrtion is not to the north edit it and direct it to the next location
                            edit(turnspeed, turntime);
                        }//else if direction is to the north turn lift 135 degree to direct the robot to the next location
                        else
                        {
                            int turntime1 = 3 * turntime;
                            turn((short)-turnspeed, (short)turntime1);
                        }
                        if (nav.backword)
                        {//if the next node is in the back of your direction just go back without rotating for saving time
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.BackMiddleSensor <= 40)
                            {//if you find and obstacle that sure will be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.BackMiddleSensor <= 40)
                            {
                                nav.direction = 1;
                                nav.obstacle = true;
                                nav.backword = false;
                            }
                            else if (nav.BackLeftSensor <= 35)
                            {//if there is a robot in lift replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 1;
                                nav.obstacle = true;
                                ob.X = cur.X-1;
                                ob.Y = cur.Y;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else if (nav.BackRightSensor <= 35)
                            {//if there is a robot in right replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 1;
                                nav.obstacle = true;
                                ob.X = cur.X;
                                ob.Y = cur.Y-1;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(1, (short)-speed, diagonaltime,true);
                                nav.backword = false;
                            }
                        }
                        else
                        {//if the next node is not in the back of your direction just go forward 
                            GetSensorsData();
                            //chick sensors to make sure that the naxt location is empty
                            if (nav.middlesensor <= 40)
                            {//if you find an obstacle it would be a robot so wait 5 seconds
                                StopToDetectTheRobotInNextPoint();
                            }
                            //after 5 seconds if the robot still in the location it mihgt be dead so break and replan the path
                            if (nav.middlesensor <= 40)
                            {
                                nav.direction = 5;
                                nav.obstacle = true;
                            }
                            else if (nav.leftsensor <= 35)
                            {//if there is a robot in lift replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 5;
                                nav.obstacle = true;
                                ob.X = cur.X;
                                ob.Y = cur.Y-1;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else if (nav.rightsensor <= 35)
                            {//if there is a robot in right replan and make it obstacle
                                //bacuse robot will crash on other robot on his way
                                Point ob = new Point();
                                nav.direction = 5;
                                nav.obstacle = true;
                                ob.X = cur.X-1;
                                ob.Y = cur.Y;
                                AddObestacle(ob);
                                nav.diagonalobstacle = true;
                            }
                            else
                            {//else if the location is empty move to it
                                MoveToNextPoint(5, speed, diagonaltime,true);
                            }
                        }
                    }
                }
            }
        }


        void InitializeMap(Point s, Point g)
        {
            map = new bool[6, 6];
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                    map[x, y] = true;

            var startLocation = s;
            var endLocation = g;
            searchParameters = new SearchParameters(startLocation, endLocation, map);
        }
        
        public void AddObestacle(Point point)
        {
            map[point.X, point.Y] = false;
        }
        
        public bool chickObestacle(Point point)
        {
            return map[point.X, point.Y];
        }
        
        
        private bool[,] map;
        private SearchParameters searchParameters;


        private void button1_Click(object sender, EventArgs e)
        {
            Point start = new Point(0, 0), goal = new Point(3, 3), ob = new Point(1, 0), ob2 = new Point(1, 1), ob3 = new Point(1, 2);
            Point current, next;
            bool done = false;
            nav.direction = 0;
            short robot = 1, speed, diognaltime, normaltime, turnspeed, turntime;
            if (robot == 1)
            {
                speed = 500;
                turnspeed = 305;
                normaltime = 275;
                diognaltime = 4400;
                turntime = 2200;
            }
            else
            {
                speed = 550;
                turnspeed = 320;
                normaltime = 275;
                diognaltime = 4400;
                turntime = 2200;
            }
            InitializeMap(start, goal);
            //  AddObestacle(ob);
            // AddObestacle(ob2);
            //  AddObestacle(ob3);
            List<Point> path;

            while (!done)
            {
                /* old
                 * Point point = new Point(2,1);
                 Program program = new Program();
                 program.InitializeMap(start, goal);
                 program.AddObestacle(point);
                 List<Point> path = program.Run(start, goal);
                 */
                searchParameters.StartLocation = start;
                PathFinder pathFinder = new PathFinder(searchParameters);
                path = pathFinder.FindPath();
                for (int z = 0; z < path.Count; z++)
                {
                    if (z == 0)
                    {
                        current = start;//to determine the source in the first movment other cases in else statment
                        next = path[z];
                    }
                    else
                    {
                        current = path[z - 1];
                        next = path[z];
                    }
                    nav.obstacle = false;
                    move(current, next, speed, turnspeed, normaltime, diognaltime, turntime);
                    if (z == path.Count - 1)
                    {
                        done = true;
                    }
                    else
                    {
                        if (nav.obstacle)
                        {
                            //if the robot found an obstacle in the next point
                            if (nav.diagonalobstacle)
                            {
                                start = current;
                                done = false;
                                break;
                            }
                            else
                            {
                                start = current;
                                AddObestacle(next);
                                done = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
        private void btnJoyStick_Click(object sender, EventArgs e)
        {

        }

        private void label30_Click(object sender, EventArgs e)
        {

        }

        private void label31_Click(object sender, EventArgs e)
        {

        }

    }
}

