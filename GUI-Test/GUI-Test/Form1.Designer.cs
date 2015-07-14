namespace GUI_Test
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.cmb_buadrate = new System.Windows.Forms.ComboBox();
            this.cmb_com = new System.Windows.Forms.ComboBox();
            this.btn_PortOpen = new System.Windows.Forms.Button();
            this.lbl_Roll = new System.Windows.Forms.TextBox();
            this.lbl_Pitch = new System.Windows.Forms.TextBox();
            this.lbl_Yaw = new System.Windows.Forms.TextBox();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.statusStrip1 = new System.Windows.Forms.StatusStrip();
            this.toolStripStatusLabel1 = new System.Windows.Forms.ToolStripStatusLabel();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.num_Vx = new System.Windows.Forms.NumericUpDown();
            this.num_Vy = new System.Windows.Forms.NumericUpDown();
            this.num_Vt = new System.Windows.Forms.NumericUpDown();
            this.num_Motion = new System.Windows.Forms.NumericUpDown();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.num_Pan = new System.Windows.Forms.NumericUpDown();
            this.num_Tillt = new System.Windows.Forms.NumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.num_delay = new System.Windows.Forms.NumericUpDown();
            this.label14 = new System.Windows.Forms.Label();
            this.groupBox1.SuspendLayout();
            this.statusStrip1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vx)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vy)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vt)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Motion)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Pan)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Tillt)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_delay)).BeginInit();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.BackColor = System.Drawing.Color.DimGray;
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.cmb_buadrate);
            this.groupBox1.Controls.Add(this.cmb_com);
            this.groupBox1.Controls.Add(this.btn_PortOpen);
            this.groupBox1.Location = new System.Drawing.Point(2, 3);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(188, 118);
            this.groupBox1.TabIndex = 110;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Com Port";
            this.groupBox1.Enter += new System.EventHandler(this.groupBox1_Enter);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 22);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(57, 13);
            this.label1.TabIndex = 5;
            this.label1.Text = "PortName:";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(6, 57);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(35, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "Baud:";
            // 
            // cmb_buadrate
            // 
            this.cmb_buadrate.FormattingEnabled = true;
            this.cmb_buadrate.Items.AddRange(new object[] {
            "9600",
            "38400",
            "57600",
            "1000000",
            "115200"});
            this.cmb_buadrate.Location = new System.Drawing.Point(80, 54);
            this.cmb_buadrate.Name = "cmb_buadrate";
            this.cmb_buadrate.Size = new System.Drawing.Size(99, 21);
            this.cmb_buadrate.TabIndex = 6;
            this.cmb_buadrate.Text = "1000000";
            // 
            // cmb_com
            // 
            this.cmb_com.FormattingEnabled = true;
            this.cmb_com.Location = new System.Drawing.Point(80, 19);
            this.cmb_com.Name = "cmb_com";
            this.cmb_com.Size = new System.Drawing.Size(99, 21);
            this.cmb_com.TabIndex = 1;
            this.cmb_com.Text = "COM4";
            this.cmb_com.SelectedIndexChanged += new System.EventHandler(this.cmb_com_SelectedIndexChanged);
            this.cmb_com.Click += new System.EventHandler(this.cmb_com_Click);
            // 
            // btn_PortOpen
            // 
            this.btn_PortOpen.Location = new System.Drawing.Point(76, 81);
            this.btn_PortOpen.Name = "btn_PortOpen";
            this.btn_PortOpen.Size = new System.Drawing.Size(105, 29);
            this.btn_PortOpen.TabIndex = 0;
            this.btn_PortOpen.Text = "Open Port";
            this.btn_PortOpen.UseVisualStyleBackColor = true;
            this.btn_PortOpen.Click += new System.EventHandler(this.btn_PortOpen_Click);
            // 
            // lbl_Roll
            // 
            this.lbl_Roll.Location = new System.Drawing.Point(557, 12);
            this.lbl_Roll.Name = "lbl_Roll";
            this.lbl_Roll.Size = new System.Drawing.Size(66, 20);
            this.lbl_Roll.TabIndex = 111;
            // 
            // lbl_Pitch
            // 
            this.lbl_Pitch.Location = new System.Drawing.Point(557, 38);
            this.lbl_Pitch.Name = "lbl_Pitch";
            this.lbl_Pitch.Size = new System.Drawing.Size(66, 20);
            this.lbl_Pitch.TabIndex = 112;
            // 
            // lbl_Yaw
            // 
            this.lbl_Yaw.Location = new System.Drawing.Point(557, 64);
            this.lbl_Yaw.Name = "lbl_Yaw";
            this.lbl_Yaw.Size = new System.Drawing.Size(66, 20);
            this.lbl_Yaw.TabIndex = 113;
            // 
            // textBox4
            // 
            this.textBox4.Location = new System.Drawing.Point(557, 93);
            this.textBox4.Name = "textBox4";
            this.textBox4.Size = new System.Drawing.Size(66, 20);
            this.textBox4.TabIndex = 114;
            // 
            // statusStrip1
            // 
            this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripStatusLabel1});
            this.statusStrip1.Location = new System.Drawing.Point(0, 124);
            this.statusStrip1.Name = "statusStrip1";
            this.statusStrip1.Size = new System.Drawing.Size(902, 22);
            this.statusStrip1.TabIndex = 115;
            this.statusStrip1.Text = "statusStrip1";
            // 
            // toolStripStatusLabel1
            // 
            this.toolStripStatusLabel1.Name = "toolStripStatusLabel1";
            this.toolStripStatusLabel1.Size = new System.Drawing.Size(0, 17);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(517, 67);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(31, 13);
            this.label5.TabIndex = 118;
            this.label5.Text = "Yaw:";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(491, 96);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(57, 13);
            this.label6.TabIndex = 119;
            this.label6.Text = "Freq Read";
            // 
            // num_Vx
            // 
            this.num_Vx.DecimalPlaces = 2;
            this.num_Vx.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.num_Vx.Location = new System.Drawing.Point(245, 13);
            this.num_Vx.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.num_Vx.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            -2147483648});
            this.num_Vx.Name = "num_Vx";
            this.num_Vx.Size = new System.Drawing.Size(75, 20);
            this.num_Vx.TabIndex = 120;
            this.num_Vx.ValueChanged += new System.EventHandler(this.numericUpDown1_ValueChanged);
            // 
            // num_Vy
            // 
            this.num_Vy.DecimalPlaces = 2;
            this.num_Vy.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.num_Vy.Location = new System.Drawing.Point(245, 39);
            this.num_Vy.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.num_Vy.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            -2147483648});
            this.num_Vy.Name = "num_Vy";
            this.num_Vy.Size = new System.Drawing.Size(75, 20);
            this.num_Vy.TabIndex = 121;
            this.num_Vy.ValueChanged += new System.EventHandler(this.num_Vy_ValueChanged);
            // 
            // num_Vt
            // 
            this.num_Vt.DecimalPlaces = 2;
            this.num_Vt.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.num_Vt.Location = new System.Drawing.Point(245, 67);
            this.num_Vt.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.num_Vt.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            -2147483648});
            this.num_Vt.Name = "num_Vt";
            this.num_Vt.Size = new System.Drawing.Size(75, 20);
            this.num_Vt.TabIndex = 122;
            this.num_Vt.ValueChanged += new System.EventHandler(this.num_Vt_ValueChanged);
            // 
            // num_Motion
            // 
            this.num_Motion.Location = new System.Drawing.Point(245, 94);
            this.num_Motion.Name = "num_Motion";
            this.num_Motion.Size = new System.Drawing.Size(75, 20);
            this.num_Motion.TabIndex = 123;
            this.num_Motion.Value = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.num_Motion.ValueChanged += new System.EventHandler(this.num_Motion_ValueChanged);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(204, 15);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(22, 13);
            this.label7.TabIndex = 124;
            this.label7.Text = "Vx:";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(204, 42);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(22, 13);
            this.label8.TabIndex = 125;
            this.label8.Text = "Vy:";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(204, 68);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(20, 13);
            this.label9.TabIndex = 126;
            this.label9.Text = "Vt:";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(198, 97);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(39, 13);
            this.label10.TabIndex = 127;
            this.label10.Text = "Motion";
            // 
            // num_Pan
            // 
            this.num_Pan.DecimalPlaces = 2;
            this.num_Pan.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.num_Pan.Location = new System.Drawing.Point(413, 12);
            this.num_Pan.Maximum = new decimal(new int[] {
            314,
            0,
            0,
            131072});
            this.num_Pan.Minimum = new decimal(new int[] {
            314,
            0,
            0,
            -2147352576});
            this.num_Pan.Name = "num_Pan";
            this.num_Pan.Size = new System.Drawing.Size(75, 20);
            this.num_Pan.TabIndex = 128;
            this.num_Pan.ValueChanged += new System.EventHandler(this.num_Pan_ValueChanged);
            // 
            // num_Tillt
            // 
            this.num_Tillt.DecimalPlaces = 2;
            this.num_Tillt.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.num_Tillt.Location = new System.Drawing.Point(413, 38);
            this.num_Tillt.Maximum = new decimal(new int[] {
            314,
            0,
            0,
            131072});
            this.num_Tillt.Minimum = new decimal(new int[] {
            314,
            0,
            0,
            -2147352576});
            this.num_Tillt.Name = "num_Tillt";
            this.num_Tillt.Size = new System.Drawing.Size(75, 20);
            this.num_Tillt.TabIndex = 129;
            this.num_Tillt.ValueChanged += new System.EventHandler(this.num_Tillt_ValueChanged);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(517, 15);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(28, 13);
            this.label3.TabIndex = 116;
            this.label3.Text = "Roll:";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(517, 41);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(34, 13);
            this.label4.TabIndex = 117;
            this.label4.Text = "Pitch:";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(367, 41);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(21, 13);
            this.label11.TabIndex = 131;
            this.label11.Text = "Tilt";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(367, 15);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(29, 13);
            this.label12.TabIndex = 130;
            this.label12.Text = "Pan:";
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 1000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(413, 93);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(66, 20);
            this.textBox1.TabIndex = 132;
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(357, 97);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(56, 13);
            this.label13.TabIndex = 133;
            this.label13.Text = "Freq Write";
            // 
            // num_delay
            // 
            this.num_delay.Location = new System.Drawing.Point(413, 68);
            this.num_delay.Maximum = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.num_delay.Name = "num_delay";
            this.num_delay.Size = new System.Drawing.Size(75, 20);
            this.num_delay.TabIndex = 134;
            this.num_delay.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(342, 74);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(65, 13);
            this.label14.TabIndex = 135;
            this.label14.Text = "Delay_Write";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(902, 146);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.num_delay);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.num_Tillt);
            this.Controls.Add(this.num_Pan);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.num_Motion);
            this.Controls.Add(this.num_Vt);
            this.Controls.Add(this.num_Vy);
            this.Controls.Add(this.num_Vx);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.statusStrip1);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.lbl_Yaw);
            this.Controls.Add(this.lbl_Pitch);
            this.Controls.Add(this.lbl_Roll);
            this.Controls.Add(this.groupBox1);
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.statusStrip1.ResumeLayout(false);
            this.statusStrip1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vx)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vy)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Vt)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Motion)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Pan)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_Tillt)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_delay)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ComboBox cmb_buadrate;
        private System.Windows.Forms.ComboBox cmb_com;
        private System.Windows.Forms.Button btn_PortOpen;
        private System.Windows.Forms.TextBox lbl_Roll;
        private System.Windows.Forms.TextBox lbl_Pitch;
        private System.Windows.Forms.TextBox lbl_Yaw;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.StatusStrip statusStrip1;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.NumericUpDown num_Vx;
        private System.Windows.Forms.NumericUpDown num_Vy;
        private System.Windows.Forms.NumericUpDown num_Vt;
        private System.Windows.Forms.NumericUpDown num_Motion;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.NumericUpDown num_Pan;
        private System.Windows.Forms.NumericUpDown num_Tillt;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.NumericUpDown num_delay;
        private System.Windows.Forms.Label label14;
    }
}

