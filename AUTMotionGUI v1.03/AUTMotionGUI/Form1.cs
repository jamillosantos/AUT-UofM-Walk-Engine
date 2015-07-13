using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;

namespace AUTMotionGUI
{
    public partial class Form1 : Form
    {
        public SerialPort SDevice;
        int[ , ] Motion = new int[10,20];
        int freg = 0;
        int freg1 = 0;

        double Roll = 0;
        double Pitch = 0;
        double Yaw = 0;

        double Vx = 0;
        double Vy = 0;
        double Vt = 0;
        byte _Motion = 0;

        byte[] out_p = new byte[13];

        byte[] _inp = new byte[100];
        public enum Walk_Prameters
        {
            No_motion = 0,
            Motion_Resolution = 2,
            Gait_Frequency = 4,
            Double_Support_Sleep = 6,
            Single_Support_Sleep = 8,

            Fly_Leg_Roll_Gain = 10,
            Fly_Leg_Pitch_Gain = 12,
            Fly_Leg_Yaw_Gain = 14,
            Fly_Leg_Y_Swing_Gain = 16,
            Fly_Leg_Step_Height_Gain = 18,

            Support_Leg_Roll_Gain = 20,
            Support_Leg_Pitch_Gain = 22,
            Support_Leg_Yaw_Gain = 24,
            Support_Leg_Y_Swing_Gain = 26,
            Support_Leg_Z_Push_Gain = 28,

            Body_Sideward_Swing_Gain = 30,
            Body_Top_down_Push_Gain = 32,

            Vx_Smoothing_Ratio = 34,
            Vy_Smoothing_Ratio = 36,
            Vt_Smoothing_Ratio = 38,

            Stabilizer_Arm_Pitch_Gain = 40,
            Stabilizer_Hip_Roll_Gain = 42,
            Stabilizer_Hip_Pitch_Gain = 44,
            Stabilizer_Knee_Gain = 46,
            Stabilizer_Foot_Pitch_Gain = 48,
            Stabilizer_Foot_Roll_Gain = 50,

            Stabilizer_COM_X_Shift_Gain = 52,
            Stabilizer_COM_Y_Shift_Gain = 54,
            Stabilizer_Hopping_X_Gain = 56,
            Stabilizer_Hopping_Y_Gain = 58,

            Gyro_Roll_Low_Pass_Gain = 60,
            Gyro_Pitch_Low_Pass_Gain = 62,

            Kalman_RMesure_Roll_Rate = 64,
            Kalman_RMesure_Pitch_Rate = 66,
            Kalman_RMesure_Yaw_Rate = 68,

            COM_X_Offset = 70,
            COM_Y_Offset = 72,
            COM_Z_Offset = 74,
            COM_Roll_Offset = 76,
            COM_Pitch_Offset = 78,
            COM_Yaw_Offset = 80,

            Left_Leg_Hip_Yaw_Offset = 82,
            Left_Leg_Hip_Roll_Offset = 84,
            Left_Leg_Hip_Pitch_Offset = 86,
            Left_Leg_Knee_Offset = 88,
            Left_Leg_Foot_Pitch_Offset = 90,
            Left_Leg_Foot_Roll_Offset = 92,

            Right_Leg_Hip_Yaw_Offset = 94,
            Right_Leg_Hip_Roll_Offset = 96,
            Right_Leg_Hip_Pitch_Offset = 98,
            Right_Leg_Knee_Offset = 100,
            Right_Leg_Foot_Pitch_Offset = 102,
            Right_Leg_Foot_Roll_Offset = 104,

            Kinematic_Lower_Leg_Len = 106,
            Kinematic_Upper_Leg_Len = 108,

            Left_Arm_Pitch_ID = 110,
            Left_Arm_Roll_ID = 111,
            Left_Arm_Elbow_ID = 112,
            Left_Hip_Yaw_ID = 113,
            Left_Hip_Roll_ID = 114,
            Left_Hip_Pitch_ID = 115,
            Left_Knee_ID = 116,
            Left_Foot_Pitch_ID = 117,
            Left_Foot_Roll_ID = 118,

            Right_Arm_Pitch_ID = 119,
            Right_Arm_Roll_ID = 120,
            Right_Arm_Elbow_ID = 121,
            Right_Hip_Yaw_ID = 122,
            Right_Hip_Roll_ID = 123,
            Right_Hip_Pitch_ID = 124,
            Right_Knee_ID = 125,
            Right_Foot_Pitch_ID = 126,
            Right_Foot_Roll_ID = 127,

            Head_Pan_ID = 128,
            Head_Tilt_ID = 129,

        };  
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            cmb_com.Items.Clear();
            cmb_com.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
        }

        //****************************************************
        /// <summary>
        /// Related to Com-port functionalities are define in this section
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        #region Com-port Configuration

        private void cmb_com_Click(object sender, EventArgs e)
        {
            cmb_com.Items.Clear();
            cmb_com.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
        }


        #endregion

        //****************************************************
        /// <summary>
        /// the functions to write and read from OpenCM9.04 are defined in this section
        /// </summary>
        /// <param name="Addrss"></param>
        /// <param name="Value"></param>
        #region Functions

        private void Read_data()
        {
            if (checkBox2.Checked)
            {
                Thread read_data = new Thread(new ParameterizedThreadStart(
                        new Action<object>((t) =>
                        {
                            SDevice.ReadTimeout = 10;
                            while (true)
                            {
                                try
                                {
                                    if (SDevice.ReadByte() == (byte)254)
                                        if (SDevice.ReadByte() == (byte)254)
                                        {
                                            for (int cnt = 0; cnt < 6; cnt++)
                                            {
                                                _inp[cnt] = (byte)SDevice.ReadByte();
                                            }

                                            Roll = ((((_inp[1] << 8) + _inp[0]) / 1000.0) - 3.14) * 57.2974;
                                            Pitch = ((((_inp[3] << 8) + _inp[2]) / 1000.0) - 3.14) * 57.2974;
                                            Yaw = ((((_inp[5] << 8) + _inp[4]) / 1000.0) - 3.14) * 57.2974;


                                            Set_Param();

                                            this.Invoke(new Action(() =>
                                            {
                                                lbl_Pitch.Text = Pitch.ToString();
                                                lbl_Roll.Text = Roll.ToString();
                                                lbl_Yaw.Text = Yaw.ToString();
                                            }));

                                            Thread.Sleep(1);
                                        }
                                    freg++;
                                }
                                catch { }

                            }
                        })));

                read_data.SetApartmentState(ApartmentState.STA);
                read_data.Start();
            }
        }

        private void Write_data()
        {
            if(checkBox2.Checked)
            {
                Thread Write_data = new Thread(new ParameterizedThreadStart(
                        new Action<object>((t) =>
                        {
                            SDevice.ReadTimeout = 10;
                            while (true)
                            {
                                try
                                {

                                    Set_Param();
                                    freg1++;
                                    Thread.Sleep((int)num_delay.Value);
                                }
                                catch { }

                            }
                        })));

                Write_data.SetApartmentState(ApartmentState.STA);
                Write_data.Start();
            }
        }

        private void Set_Param()
        {
            Vx = (byte)(100 + ((double)num_Vx.Value * 100));
            Vy = (byte)(100 + ((double)num_Vy.Value * 100));
            Vt = (byte)(100 + ((double)num_Vt.Value * 100));
            _Motion = (byte)num_Motion.Value;

            out_p[0] = 254;
            out_p[1] = (byte)Vx;
            out_p[2] = (byte)Vy;
            out_p[3] = (byte)Vt;
            out_p[4] = _Motion;
            out_p[5] = (byte)((((double)num_Pan.Value * 100) / 3.1415) + 100);
            out_p[6] = (byte)((((double)num_Tillt.Value * 100) / 3.1415) + 100);

            // textBox4.Text = freg.ToString();
            try
            {
                SDevice.Write(out_p, 0, 7);
            }
            catch
            { }
        }

        private void Set_Walk_Param(byte Addrss, ushort Value)
        {
            byte[] _tmp = new byte[6];
            _tmp[0] = 254;
            _tmp[1] = 254;
            _tmp[2] = 101;
            _tmp[3] = Addrss;
            _tmp[4] = (byte)Value;
            _tmp[5] = (byte)(Value >> 8);
            if (SDevice.IsOpen)
                SDevice.Write(_tmp, 0, 6);
            System.Threading.Thread.Sleep(10);
        }

        private double Walk_Get_Para_D(byte Address)
        {
            byte[] tmp = new byte[2];
            byte[] _tmp = new byte[4];

            _tmp[0] = 254;
            _tmp[1] = 254;
            _tmp[2] = 102;
            _tmp[3] = (byte)Address;
            try
            {
                SDevice.Write(_tmp, 0, 4);
            }
            catch { MessageBox.Show("error in write to port!"); }
            System.Threading.Thread.Sleep(10);

            SDevice.ReadTimeout = 1000;
            if ((byte)SDevice.ReadByte() == 254)
            {
                if ((byte)SDevice.ReadByte() == 254)
                {
                    if ((byte)SDevice.ReadByte() == 102)
                    {
                        if ((byte)SDevice.ReadByte() == Address)
                        {
                            tmp[0] = (byte)SDevice.ReadByte();
                            tmp[1] = (byte)SDevice.ReadByte();
                        }
                    }
                }
            }

            ushort T = (ushort)((tmp[1] << 8) + tmp[0]);
            double D = 0;
            if (T > 32767)
            {
                D = (double)(((T - 32767) / 1000.0) * -1.0);
            }
            else
            {
                D = (double)(T / 1000.0);
            }
            System.Threading.Thread.Sleep(150);
            return D;
        }

        private void Get_Page(byte Page_Num)
        {
            byte[] tmp = new byte[23];
            byte[] _tmp = new byte[4];

            _tmp[0] = 255;
            _tmp[1] = 255;
            _tmp[2] = 150;
            _tmp[3] = (byte)Page_Num;

            try
            {
                SDevice.Write(_tmp, 0, 4);
            }
            catch { MessageBox.Show("error in write to port!"); }

            System.Threading.Thread.Sleep(10);

            SDevice.ReadTimeout = 1000;
            if ((byte)SDevice.ReadByte() == 255)
            {
                if ((byte)SDevice.ReadByte() == 255)
                {
                    if ((byte)SDevice.ReadByte() == 150)
                    {
                        if ((byte)SDevice.ReadByte() == Page_Num)
                        {
                            for (byte cnt = 0; cnt < 20; cnt++)
                            {
                                Motion[Page_Num, cnt] = (byte)SDevice.ReadByte();
                            }
                        }
                    }
                }
            }
        }

        private Int16 Walk_Get_Para_I(byte Address)
        {
            byte[] tmp = new byte[2];
            byte[] _tmp = new byte[4];

            _tmp[0] = 254;
            _tmp[1] = 254;
            _tmp[2] = 102;
            _tmp[3] = (byte)Address;
            try
            {
                SDevice.Write(_tmp, 0, 4);
            }
            catch { MessageBox.Show("error in write to port!"); }

            SDevice.ReadTimeout = 1000;
            if ((byte)SDevice.ReadByte() == 254)
            {
                if ((byte)SDevice.ReadByte() == 254)
                {
                    if ((byte)SDevice.ReadByte() == 102)
                    {
                        if ((byte)SDevice.ReadByte() == Address)
                        {
                            tmp[0] = (byte)SDevice.ReadByte();
                            tmp[1] = (byte)SDevice.ReadByte();
                        }
                    }
                }
            }

            ushort T = (ushort)((tmp[1] << 8) + tmp[0]);
            Int16 D = 0;
            if (T > 32767)
            {
                D = (Int16)(((T - 32767)) * -1.0);
            }
            else
            {
                D = (Int16)(T);
            }
            System.Threading.Thread.Sleep(10);
            if ((D >= -1000) && (D <= 1000)) return D;
            return 0;
        }

        private void Set_Walk(double Vx, double Vy, double Vt, byte Motion_Num)
        {
            byte[] _tmp = new byte[12];

            _tmp[0] = 254;
            _tmp[1] = 254;
            _tmp[2] = 100;

            ushort T = Double_To_UShort(Vx);

            _tmp[3] = (byte)(T);
            _tmp[4] = (byte)(T >> 8);

            T = Double_To_UShort(Vy);
            _tmp[5] = (byte)(T);
            _tmp[6] = (byte)(T >> 8);

            T = Double_To_UShort(Vt);
            _tmp[7] = (byte)(T);
            _tmp[8] = (byte)(T >> 8);

            _tmp[9] = Motion_Num;

            if (SDevice.IsOpen)
                SDevice.Write(_tmp, 0, 10);
        }

        private ushort Double_To_UShort(double data)
        {
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            return T;
        }

        #endregion

        //****************************************************
        /// <summary>
        /// The function related to forms abjects and Events are defined in this section
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        #region Form actions
        private void btn_PortOpen_Click(object sender, EventArgs e)
        {
            if (SDevice == null)
            {
                if (cmb_com.SelectedIndex == -1)
                {
                    toolStripStatusLabel1.Text = "You must select one item from Combo-Box!!";
                }
                else
                {
                    SDevice = new SerialPort(cmb_com.Text, Convert.ToInt32(cmb_buadrate.Text));
                    if (SDevice.IsOpen == false)
                    {
                        try
                        {
                            SDevice.Open();
                            toolStripStatusLabel1.Text = "Port is Open";
                            btn_PortOpen.Text = "Close Port";
                        }
                        catch
                        {
                            toolStripStatusLabel1.Text = "ERR... the ports is not initialize!";
                        }
                    }
                }
            }
            else
            {
                if (btn_PortOpen.Text == "Close Port")
                {
                    try
                    {
                        SDevice.Close();
                        toolStripStatusLabel1.Text = "Port is Closed";
                        btn_PortOpen.Text = "Open Port";
                    }
                    catch
                    {
                        toolStripStatusLabel1.Text = "ERR... the ports is not initialize!";
                    }
                }
                else 
                {
                    if (btn_PortOpen.Text == "Open Port")
                    {
                        try
                        {
                            SDevice.Open();
                            toolStripStatusLabel1.Text = "Port is Open";
                            btn_PortOpen.Text = "Close Port";
                        }
                        catch
                        {
                            toolStripStatusLabel1.Text = "ERR... the ports is not initialize!";
                        }
                    }
                }
            }
        }

        private void tabControl2_Selected(object sender, TabControlEventArgs e)
        {
           
        }

        private void tabPage1_Enter(object sender, EventArgs e)
        {
            //read value of data
            Nud_MR.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Motion_Resolution);         
            Nud_GF.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Gait_Frequency);
            Nud_DSS.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Double_Support_Sleep);
            Nud_SSS.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Single_Support_Sleep);
            Nud_FLSHGa.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Fly_Leg_Step_Height_Gain);
            Nud_SLZPG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Support_Leg_Z_Push_Gain);
            Nud_BSSG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Body_Sideward_Swing_Gain);
            Nud_FLYSG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Fly_Leg_Y_Swing_Gain);
            Nud_FLRG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Fly_Leg_Roll_Gain);
            Nud_SLRG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Support_Leg_Roll_Gain);
            Nud_SLYSG.Value = (decimal)Walk_Get_Para_D((byte)Walk_Prameters.Support_Leg_Y_Swing_Gain);
        }

        private void tabPage1_Click(object sender, EventArgs e)
        {

        }

        private void Nud_MR_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_MR.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Motion_Resolution, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_GF_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_GF.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Gait_Frequency, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_DSS_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_DSS.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Double_Support_Sleep, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_SSS_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SSS.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Single_Support_Sleep, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_KLLL_ValueChanged(object sender, EventArgs e)
        {

        }

        private void Nud_FLSHGa_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_FLSHGa.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Fly_Leg_Step_Height_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_SLZPG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SLZPG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Support_Leg_Z_Push_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_BSSG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_BSSG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Body_Sideward_Swing_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_FLYSG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_FLYSG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Fly_Leg_Y_Swing_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_FLRG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_FLRG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Fly_Leg_Roll_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_SLRG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SLRG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Support_Leg_Roll_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_Vx_ValueChanged(object sender, EventArgs e)
        {
            double Vx=(double)Nud_Vx.Value;
            double Vy=(double)Nud_Vy.Value;
            double Vt=(double)Nud_Vt.Value;
            byte Motion=100;
            Set_Walk(Vx, Vy, Vt, Motion);
            System.Threading.Thread.Sleep(100);
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            
        }

        private void button2_Click(object sender, EventArgs e)
        {
            //label30.Text = ((decimal)Walk_Get_Para_D((byte)72)).ToString(); 
        }

        private void tabPage10_Click(object sender, EventArgs e)
        {

        }

        private void nud_RAPO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)nud_RAPO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)136, T);
            System.Threading.Thread.Sleep(100);
        }

        private void nud_LAPO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)nud_LAPO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)142, T);
            System.Threading.Thread.Sleep(100);
        }

        private void numericUpDown3_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)nud_RAEO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)140, T);
            System.Threading.Thread.Sleep(100);
        }

        private void numericUpDown2_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)nud_LAEO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)146, T);
            System.Threading.Thread.Sleep(100);
        }

        private void tabPage10_Enter(object sender, EventArgs e)
        {
            //0000
            Nud_CXO.Value = (decimal)Walk_Get_Para_I(70);
            Nud_CYO.Value = (decimal)Walk_Get_Para_I(72);
            Nud_CZO.Value = (decimal)Walk_Get_Para_I(74);

            Nud_RLHPO.Value = (decimal)Walk_Get_Para_D(98);
            Nud_LLHPO.Value = (decimal)Walk_Get_Para_D(86);
        }

        private void tabPage6_Enter(object sender, EventArgs e)
        {
            //-----
            Nud_SAPG.Value = (decimal)Walk_Get_Para_D(40);
            Nud_SHPG.Value = (decimal)Walk_Get_Para_D(44);

            Nud_SCXSG.Value = (decimal)Walk_Get_Para_D(52);
        }

        private void Nud_SAPG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SAPG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)40, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_SHPG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SHPG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)44, T);
            System.Threading.Thread.Sleep(100);
        }

        private void tabPage6_Click(object sender, EventArgs e)
        {

        }

        private void Nud_CYO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_CYO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data)) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data);
            }
            Set_Walk_Param((byte)72, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_CXO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_CXO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data)) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data);
            }
            Set_Walk_Param((byte)70, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_CZO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_CZO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data)) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data);
            }
            Set_Walk_Param((byte)74, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_SCXSG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SCXSG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)52, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_Head_Pan_ValueChanged(object sender, EventArgs e)
        {

        }

        private void Nud_Head_Tilt_ValueChanged(object sender, EventArgs e)
        {

        }

        private void Nud_SLYSG_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_SLYSG.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Support_Leg_Y_Swing_Gain, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_RLHPO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_RLHPO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Right_Leg_Hip_Pitch_Offset, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Nud_LLHPO_ValueChanged(object sender, EventArgs e)
        {
            double data = (double)Nud_LLHPO.Value;
            ushort T = 0;
            if (data < 0)
            {
                T = (ushort)(((-1 * data) * 1000) + 32767);
            }
            else if (data >= 0)
            {
                T = (ushort)(data * 1000);
            }
            Set_Walk_Param((byte)Walk_Prameters.Left_Leg_Hip_Pitch_Offset, T);
            System.Threading.Thread.Sleep(100);
        }

        private void Btn_Get_Page_Click(object sender, EventArgs e)
        {
            Get_Page(byte.Parse(lblb_PageNum.SelectedValue.ToString()));
        }

        #endregion

        private void checkBox2_CheckStateChanged(object sender, EventArgs e)
        {
            if (checkBox2.Checked)
            {
                Thread read_data = new Thread(new ParameterizedThreadStart(
                        new Action<object>((t) =>
                        {
                            SDevice.ReadTimeout = 10;
                            while (true)
                            {
                                try
                                {
                                    if (SDevice.ReadByte() == (byte)254)
                                        if (SDevice.ReadByte() == (byte)254)
                                        {
                                            for (int cnt = 0; cnt < 6; cnt++)
                                            {
                                                _inp[cnt] = (byte)SDevice.ReadByte();
                                            }

                                            Roll = ((((_inp[1] << 8) + _inp[0]) / 1000.0) - 3.14) * 57.2974;
                                            Pitch = ((((_inp[3] << 8) + _inp[2]) / 1000.0) - 3.14) * 57.2974;
                                            Yaw = ((((_inp[5] << 8) + _inp[4]) / 1000.0) - 3.14) * 57.2974;


                                            Set_Param();

                                            this.Invoke(new Action(() =>
                                            {
                                                lbl_Pitch.Text = Pitch.ToString();
                                                lbl_Roll.Text = Roll.ToString();
                                                lbl_Yaw.Text = Yaw.ToString();
                                            }));

                                            Thread.Sleep(1);
                                        }
                                    freg++;
                                }
                                catch { }

                            }
                        })));

                read_data.SetApartmentState(ApartmentState.STA);
                read_data.Start();
            }
        }

        private void timer1_Tick_1(object sender, EventArgs e)
        {
            textBox4.Text = freg.ToString();
            tbt_Freq.Text = freg1.ToString();
            freg = 0;
            freg1 = 0;
        }

        //****************************************************
    }
}
