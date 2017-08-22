using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Media.Media3D;
using System.Windows.Media.Animation;
//using Viewport3D;
//using Microsoft.Win32;
using ImuDataPolling;

namespace WpfApplication1
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {


        public static Double w = 0, x = 0, y = 0, z = 0, angle = 0;


        public MainWindow()
        {
            InitializeComponent();
            
            ImuDataPolling.ReadIsochronous.ImuDataUpdate += new ImuDataPolling.ReadIsochronous.ImuDataHandler(ImuData);
            ImuDataPolling.ReadIsochronous.ImuDataThreadStart();
/*
            OpenFileDialog ofd = new OpenFileDialog();

            

                WavefrontObjLoader wfl = new WavefrontObjLoader();

                Model3DGroup myModel3DGroup = new Model3DGroup();

                DirectionalLight myDirectionalLight = new DirectionalLight();

                myDirectionalLight.Color = Colors.White;

                myDirectionalLight.Direction = new Vector3D(-0.61, -0.5, -0.61);

                myModel3DGroup.Children.Add(myDirectionalLight);

                var m = wfl.LoadObjFile(@"G:\code\STM32\Kalman\飞行姿态模拟\3d飞控的版本\WpfApplication1\bin\Debug\hanlanda_.obj");

                m.Content = myModel3DGroup;

                test.Children.Add(m);
*/
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
           
        }
        void func()///欧拉到四元的转换，然后传给rotate
        {
            w = Math.Cos(Roll.Value / 2) * Math.Cos(Pitch.Value / 2) * Math.Cos(Yaw.Value/2) - Math.Sin(Roll.Value / 2) * Math.Sin(Pitch.Value / 2) * Math.Sin(Yaw.Value/2);
            x = Math.Sin(Roll.Value / 2) * Math.Cos(Pitch.Value / 2) * Math.Cos(Yaw.Value/2) - Math.Cos(Roll.Value / 2) * Math.Sin(Pitch.Value / 2) * Math.Sin(Yaw.Value/2);
            y = Math.Cos(Roll.Value / 2) * Math.Sin(Pitch.Value / 2) * Math.Cos(Yaw.Value/2) - Math.Sin(Roll.Value / 2) * Math.Cos(Pitch.Value / 2) * Math.Sin(Yaw.Value/2);
            z = Math.Cos(Roll.Value / 2) * Math.Cos(Pitch.Value / 2) * Math.Sin(Yaw.Value/2) - Math.Sin(Roll.Value / 2) * Math.Sin(Pitch.Value / 2) * Math.Cos(Yaw.Value/2);

            angle = 57.3 * 2 * Math.Acos(w);
        }

        private void slider1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            func();
            Vector3D pitch = new Vector3D(x, y, z);
            rotate.Axis = pitch;
            rotate.Angle = angle;
        }

        private void slider2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            func();
            Vector3D pitch = new Vector3D(x, y, z);
            rotate.Axis = pitch;
            rotate.Angle = angle;
        }
        
        private void slider3_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            func();
            Vector3D pitch = new Vector3D(x, y, z);
            rotate.Axis = pitch;
            rotate.Angle = angle;
        }

        public Model3D myDirectionalLight { get; set; }
        public void ImuData(Double imu_x, Double imu_y, Double imu_z, Double imu_w)
        {
            rotate.Dispatcher.Invoke(new Action(() =>
            {
                x = imu_x;
                y = imu_y;
                z = imu_z;
                w = imu_w;
                //Console.WriteLine("ImuData  x:{0},y:{1},z:{2},w:{3}", x, y, z, w);
                angle = 57.3 * 2 * Math.Acos(w);
                Vector3D pitch = new Vector3D(x, y, z);
                rotate.Axis = pitch;
                rotate.Angle = angle;
            }));

        }
    }
}
