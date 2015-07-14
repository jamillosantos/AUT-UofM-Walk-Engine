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
using System.IO;

namespace GUI_Test
{
    public partial class Form1 : Form
    {

        SerialPort SD_Com;

        int freg = 0;
        int freg1 = 0;

        double Roll = 0;
        double Pitch = 0;
        double Yaw = 0;

        double Vx = 0;
        double Vy = 0;
        double Vt = 0;
        byte Motion = 0;

        byte[] out_p=new byte[13];

        byte[] _inp = new byte[100];
        public Form1()
        {
            InitializeComponent();
        }

        private void btn_PortOpen_Click(object sender, EventArgs e)
        {
            SD_Com = new SerialPort(cmb_com.Text, Convert.ToInt32(cmb_buadrate.Text));
            if (SD_Com.IsOpen == false)
            {
                try
                {
                    SD_Com.ReadTimeout = 100;
                    SD_Com.Open();
                    btn_PortOpen.Text = "Close Port";
                    cmb_com.Text = SD_Com.PortName.ToString();
                    toolStripStatusLabel1.Text = "Port is Open";
                    Read_data();
                    //Write_data();
                }
                catch
                {
                    toolStripStatusLabel1.Text = "Port did not initialize!";
                    Console.Beep();
                }
            }
            else
            {
                try
                {
                    SD_Com.Close();
                    btn_PortOpen.Text = "Open Port";
                    cmb_com.Text = SD_Com.PortName.ToString();
                    toolStripStatusLabel1.Text = "Port is close";
                }
                catch
                {
                    toolStripStatusLabel1.Text = "Port did not initialize!";
                    Console.Beep();
                }
            }
            
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            cmb_com.Items.Clear();
            cmb_com.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
        }

        private void Read_data()
        {
            Thread read_data = new Thread(new ParameterizedThreadStart(
                    new Action<object>((t) =>
                    {
                        SD_Com.ReadTimeout = 10;
                        while (true)
                        {
                            try
                            {
                                if (SD_Com.ReadByte() == (byte)254)
                                    if (SD_Com.ReadByte() == (byte)254)
                                    {
                                        for (int cnt = 0; cnt < 6; cnt++)
                                        {
                                            _inp[cnt] = (byte)SD_Com.ReadByte();
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

        private void Write_data()
        {
            Thread Write_data = new Thread(new ParameterizedThreadStart(
                    new Action<object>((t) =>
                    {
                        SD_Com.ReadTimeout = 10;
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


        private void cmb_com_Click(object sender, EventArgs e)
        {
            cmb_com.Items.Clear();
            cmb_com.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void Set_Param()
        {
            Vx = (byte)(100 + ((double)num_Vx.Value * 100));
            Vy = (byte)(100 + ((double)num_Vy.Value * 100));
            Vt = (byte)(100 + ((double)num_Vt.Value * 100));
            Motion = (byte)num_Motion.Value;

            out_p[0] = 254;
            out_p[1] = (byte)Vx;
            out_p[2] = (byte)Vy;
            out_p[3] = (byte)Vt;
            out_p[4] = Motion;
            out_p[5] = (byte)((((double)num_Pan.Value * 100) / 3.1415) + 100);
            out_p[6] = (byte)((((double)num_Tillt.Value * 100) / 3.1415) + 100);

           // textBox4.Text = freg.ToString();
            try
            {
                SD_Com.Write(out_p, 0, 7);
            }
            catch
            { }
        }

        private void num_Pan_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void num_Tillt_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void num_Vy_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void num_Vt_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void num_Motion_ValueChanged(object sender, EventArgs e)
        {
            Set_Param();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {

            textBox4.Text = freg.ToString();
            textBox1.Text = freg1.ToString();
            freg = 0;
            freg1 = 0;
        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void cmb_com_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}
