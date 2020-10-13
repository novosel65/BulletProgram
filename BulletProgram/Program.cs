using System;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using System.IO;
using System.Reflection;
using System.Drawing;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using Emgu.CV.ImgHash;
using System.Runtime.InteropServices.ComTypes;
using System.ComponentModel.DataAnnotations;

namespace BulletProgram
{
    /// <summary>
    /// Class Profile
    /// profile - профиль
    /// inf - информативность профиля
    /// alfa - угол наклона нареза (к которому относится профиль)
    /// land - поле нарезов, которому принадлежит профиль
    /// coor - вспомогательная переменная для хранения списка корелляций для профиля
    /// </summary>
    class Profile
    {
        public List<double> profile { get; set; }
        public Point p1 { get; set; }
        public Point p2 { get; set; }
        public double inf { get; set; }
        public double alfa { get; set; }
        public double[] corr { get; set; }
        public int land { get; set; }
        public Profile(List<double> profile, Point p1, Point p2, double alfa)
        {
            this.profile = profile;
            this.p1 = p1;
            this.p2 = p2;
            this.alfa = alfa;
            this.inf = -1;
        }
        /// <summary>
        /// Функция расчета информативности профиля
        /// </summary>
        public void CalcInfo()
        {
            double res = BulletFun.Sd(profile);
            this.inf = res;
        }
    }

    /// <summary>
    /// Class BulletFun
    /// Библиотека функций
    /// </summary>
    class BulletFun
    {
        /// <summary>
        /// Mean
        /// Расчет среднего значения
        /// </summary>
        public static double Mean(List<double> table)
        {
            double total = 0;

            foreach (double value in table)
            {
                total += value;
            }
            return total / table.Count;
        }
        /// <summary>
        /// Sd
        /// Расчет стандартного отклонения
        /// </summary>
        public static double Sd(List<double> table)
        {

            double mean = Mean(table);

            double squareSum = 0;

            foreach (double value in table)
            {

                squareSum += Math.Pow(value - mean, 2);

            }

            //return Math.Sqrt((squareSum) / (table.Count - 1));
            return Math.Sqrt((squareSum) / (table.Count));

        }
        /// <summary>
        /// GetIntersection
        /// проверка на пересечение двух зон одного типа
        /// </summary>
        public static int GetIntersection(Profile profile1, Profile profile2)
        {
            int start = Math.Max(profile1.p1.Y, profile2.p1.Y);
            int end = Math.Min(profile1.p2.Y, profile2.p2.Y);
            return end - start;
        }

        //public static int LinQIndexOfMaximumElement(IList<double> list)
        //{
        //    int maxIndex = list.Select((x, i) => new { Value = x, Index = i }).Aggregate
        //    (
        //        new { Value =double.MinValue, Index = -1 },
        //        (a, x) => (a.Index < 0) || (x.Value > a.Value) ? x : a,
        //        a => a.Index
        //    );
        //    return maxIndex;
        //}

        /// <summary>
        /// IndexOfMaximumElement
        /// расчет индекса максимального элемента
        /// </summary>
        public static int IndexOfMaximumElement(IList<double> list)
        {
            int size = list.Count;

            if (size < 2)
                return size - 1;

            double maxValue = list[0];
            int maxIndex = 0;

            for (int i = 1; i < size; ++i)
            {
                double thisValue = list[i];
                if (thisValue > maxValue)
                {
                    maxValue = thisValue;
                    maxIndex = i;
                }
            }

            return maxIndex;
        }

        /// <summary>
        /// GetCorrelation
        /// расчет коэффициента корелляции
        /// </summary>
        public static double GetCorrelation(List<double> x, List<double> y)
        {
            if (x.Count != y.Count)
                throw new Exception("Length of sources is different");
            double avgX = Mean(x);
            double stdevX = Sd(x);
            double avgY = Mean(y);
            double stdevY = Sd(y);
            double covXY = 0;
            int len = x.Count;
            for (int i = 0; i < len; i++)
                covXY += (x[i] - avgX) * (y[i] - avgY);
            covXY /= len;
            double pearson = covXY / (stdevX * stdevY);
            return pearson;
        }
        /// <summary>
        /// GetCorrelationAuto
        /// расчет коэффициента корелляции без нормализации
        /// </summary>
        public static double GetCorrelationAuto(List<double> x, List<double> y)
        {
            if (x.Count != y.Count)
                throw new Exception("Length of sources is different");
            double covXY = 0;
            int len = x.Count;
            for (int i = 0; i < len; i++)
                covXY += x[i] * y[i];
            double pearson = covXY;
            return pearson;
        }

        /// <summary>
        /// GetCorrelationOfSeries
        /// расчет значений кросс-корелляции двух профилей разной длины
        /// </summary>
        public static List<double> GetCorrelationOfSeries(List<double> x, List<double> y, string mode)
        {
            List<double> autoCorrelation = new List<double>();
            int len;
            List<double> vrem = new List<double>();
            List<double> xNew = x.ToList();
            List<double> xRange = new List<double>();
            switch (mode)
            {
                case "full":
                    len = x.Count + y.Count - 1;
                    //int capacity;
                    //var list = Enumerable.Range(0, capacity).Select(i => 0d).ToList();
                    vrem = Enumerable.Repeat(0d, y.Count - 1).ToList();
                    vrem.ForEach(item => xNew.Insert(0, item));
                    vrem.ForEach(item => xNew.Add(item));
                    for (int i = 0; i < len; i++)
                    {
                        xRange = xNew.GetRange(i, y.Count);
                        autoCorrelation.Add(GetCorrelationAuto(xRange, y));
                    }
                    break;
                case "valid":
                    len = Math.Max(x.Count, y.Count) - Math.Min(x.Count, y.Count) + 1;
                    if (x.Count < y.Count)
                    {
                        vrem = Enumerable.Repeat(0d, len - 1).ToList();
                        vrem.ForEach(item => xNew.Insert(0, item));
                        vrem.ForEach(item => xNew.Add(item));
                    }
                    for (int i = 0; i < len; i++)
                    {
                        xRange = x.GetRange(i, y.Count);
                        autoCorrelation.Add(GetCorrelationAuto(xRange, y));
                    }
                    break;
            }
            return autoCorrelation;
        }

        /// <summary>
        /// Calc_cross
        /// расчет оптимального (максимального) значения кросс-корелляции двух профилей
        ///  (с их оптимальным выравниванием)
        /// </summary>
        public static double Calc_cross(Profile profile1, Profile profile2)
        {
            bool flagw = false;// флаг для варианта расчета
            double mean1 = Mean(profile1.profile);
            double sd1 = Sd(profile1.profile);
            double mean2 = Mean(profile2.profile);
            double sd2 = Sd(profile2.profile);
            double coef_max;
            // нормализация профилей
            List<double> pr1 = new List<double>();
            if (sd1 == 0)
            {
                pr1.AddRange(profile1.profile);
            }
            else
                profile1.profile.ForEach(item => pr1.Add((item - mean1) / sd1));
            List<double> pr2 = new List<double>();
            if (sd2 == 0)
            {
                pr2.AddRange(profile2.profile);
            }
            else
                profile2.profile.ForEach(item => pr2.Add((item - mean2) / sd2));
            //--------------------два варианта расчета-------------
            if (flagw)
            {
                List<double> a1 = GetCorrelationOfSeries(pr1, pr2, "valid");
                int ind;
                if (pr1.Count >= pr2.Count)
                {
                    coef_max = a1[0] / Math.Min(pr1.Count, pr2.Count);
                    ind = 0;
                }
                else
                {
                    coef_max = a1[^1] / Math.Min(pr1.Count, pr2.Count);
                    ind = pr2.Count - pr1.Count;
                }
            }
            else
            {
                int[] rvalues;
                double coef;
                double par = 0.95;
                int sh1 = pr1.Count;
                int sh2 = pr2.Count;
                if (sh1 >= sh2)
                {
                    rvalues = Enumerable.Range(-(sh2 - 1), sh1 + sh2 - 1).ToArray();
                    coef = (double)sh2 / sh1;
                }
                else
                {
                    rvalues = Enumerable.Range(-(sh2 - 1), sh1 + sh2 - 1).ToArray();
                    coef = (double)sh1 / sh2;
                }
                // расчет всех значений кросс корелляции двух профилей различной длины со смещениями
                List<double> corr = GetCorrelationOfSeries(pr1, pr2, "full");

                int t1, t2;


                if (sh1 >= sh2)
                {
                    t2 = (int)Math.Round(pr2.Count * (1 - par));
                    t1 = pr2.Count - t2;
                }
                else
                {
                    t2 = (int)Math.Round(pr1.Count * (1 - par));
                    t1 = pr2.Count - t2;
                }
                List<double> corr1 = corr.GetRange(t1 - 1, 2 * t2 + 1);  // 2*60
                //int ind1 = IndexOfMaximumElement(corr1);
                //ind1 = ind1 + t1 - 1;
                double cor_max = corr1.Max();
                int ind = corr.IndexOf(cor_max);
                // значение оптимального сдвига для первого профиля
                int lag = rvalues[ind];
                int lag_max = lag;
                lag = Math.Abs(lag);
                double a1;
                double a2;
                int amin;
                // расчет оптимального значения корелляции после сдвига
                if (lag == 0)
                {
                    amin = Math.Min(pr1.Count, pr2.Count);
                    a1 = GetCorrelation(pr1.GetRange(0, amin), pr2.GetRange(0, amin));
                    a2 = a1;
                }
                else
                {
                    if (rvalues[ind] < 0)
                    {
                        if ((pr2.Count - lag) > pr1.Count)
                        {
                            a1 = GetCorrelation(pr2.GetRange(lag, pr1.Count), pr1);
                        }
                        else
                        {
                            a1 = GetCorrelation(pr2.GetRange(lag, pr2.Count - lag), pr1.GetRange(0, pr2.Count - lag));
                        }
                    }
                    else
                    {
                        if ((pr1.Count - lag) > pr2.Count)
                        {
                            a1 = GetCorrelation(pr1.GetRange(lag, pr2.Count), pr2);
                        }
                        else
                        {
                            a1 = GetCorrelation(pr1.GetRange(lag, pr1.Count - lag), pr2.GetRange(0, pr1.Count - lag));
                            a1 = GetCorrelation(pr1.GetRange(lag, pr1.Count - lag), pr2.GetRange(0, pr1.Count - lag));
                        }
                    }
                }

                coef_max = a1 * coef;
            }
            return coef_max;
        }

        /// <summary>
        /// Bresenham_march
        /// определение значений на изображении между двумя точками
        /// с учетом угла наклона
        /// </summary>
        public static ArrayList Bresenham_march(Image<Gray, Byte> img, Point p1, Point p2)
        {
            int x1 = p1.X;
            int y1 = p1.Y;
            int x2 = p2.X;
            int y2 = p2.Y;
            ArrayList ret = new ArrayList();
            //tests if any coordinate is outside the image
            if (x1 >= img.Width || x2 >= img.Width || y1 >= img.Height || y2 >= img.Height)
            {
                //# tests if line is in image, necessary because some part of the line must be inside, it respects the case that
                bool res = CvInvoke.ClipLine(new Rectangle(0, 0, img.Width, img.Height), ref p1, ref p2);
                if (!res)
                {
                    Console.WriteLine("not in region");
                    return ret;
                }
            }
            bool steep = Math.Abs(y2 - y1) > Math.Abs(x2 - x1);
            if (steep)
            {
                (x1, y1) = (y1, x1);
                (x2, y2) = (y2, x2);
            }
            // takes left to right
            bool also_steep = x1 > x2;
            if (also_steep)
            {
                (x1, x2) = (x2, x1);
                (y1, y2) = (y2, y1);
            }
            int dx = x2 - x1;
            int dy = Math.Abs(y2 - y1);
            double error = 0.0;
            double delta_error = 0.0;
            // Default if dx is zero
            if (dx != 0) delta_error = Math.Abs((double)dy / dx);
            int y_step = y1 < y2 ? 1 : -1;
            int y = y1;

            foreach (int x in Enumerable.Range(x1, x2 - x1))
            {
                (int, int) p = steep ? (y, x) : (x, y);
                if (p.Item1 < img.Height && p.Item2 < img.Width)
                {
                    ret.Add((p, img.Data[p.Item1, p.Item2, 0]));
                }
                error += delta_error;
                if (error >= 0.5)
                {
                    y += y_step;
                    error -= 1;
                }
            }
            if (also_steep) ret.Reverse();// because we took the left to right instead
            return ret;
        }

        /// <summary>
        /// Calc_profile
        /// расчет значений профиля зоны изображения
        /// img - изображение
        /// p1 - координаты левого верхнего угла зоны
        /// p2 - координаты правого нижнего угла зоны
        /// alfa - угол наклона (нареза)
        /// path - путь у файлу изображения
        /// land - поле нарезов которой принадлежит зона (профиль)
        /// pr - порядковый номер профиля
        /// </summary>
        public static (Profile, double) Calc_profile(Image<Bgr, Byte> img, Point p1, Point p2, double alfa, string path,
            int land, int pr)
        {
            Image<Gray, Byte> gray = img.Convert<Gray, Byte>();
            double centr = (p2.Y - p1.Y) / 2;
            double xdelta = centr * alfa;
            Point pp1 = new Point(p2.Y, 0);
            Point pp2 = new Point(p1.Y, 0);
            List<Point> pp = new List<Point>
                    {
                        new Point(p2.Y, (int)Math.Round(p1.X - xdelta)),
                        new Point(p1.Y, (int)Math.Round(p1.X + xdelta))
                    };

            // prove the angle
            double tgalfa = (pp[0].X - pp[1].X) / (pp[1].Y - pp[0].Y);
            // find the mean value along the lines - profile
            List<double> profile = new List<double>();
            ArrayList coor = new ArrayList();
            int sum;
            foreach (int value in Enumerable.Range(p1.X, p2.X - p1.X))
            {
                sum = 0;
                pp1.Y = (int)Math.Round(value - xdelta);
                pp2.Y = (int)Math.Round(value + xdelta);
                ArrayList res = Bresenham_march(gray, pp1, pp2);
                if (res.Count > 0)
                {
                    List<(int, int)> result = new List<(int, int)>();
                    foreach (((int, int), Byte) i in res)
                    {
                        sum += Convert.ToInt32(i.Item2);
                        result.Add(i.Item1);
                    }
                    double all_sum = (double)sum / res.Count;
                    profile.Add(all_sum);
                    coor.Add(result);
                }
            }
            Profile prof = new Profile(profile, p1, p2, alfa);
            prof.CalcInfo();
            return (prof, centr);
        }
    }

    /// <summary>
    /// class Program
    /// для инициализации параметров и запуска функций
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            //Image<Bgr, Byte> img1 = new Image<Bgr, Byte>("MyImage.jpg");
            string newPath = Path.Combine(
                    Path.GetDirectoryName(Assembly.GetEntryAssembly().Location)
               , "Images_07\\1118.000135.04");
            string[] files = Directory.GetFiles(newPath, "*.jpg");

            ArrayList profiles = new ArrayList();
            //List<Point> points = new List<Point>();
            Point p1 = new Point();
            Point p2 = new Point();
            foreach (string file in files)
            {
                // Загрузка изображения из файла
                Image<Bgr, Byte> img1 = new Image<Bgr, Byte>(file);
                Image<Bgr, Byte> img = img1.Flip(FlipType.Vertical);
                Size tt = img.Size;
                // инициализация значений углов для расчета профилей двух зон (из разных полей нарезов)
                double[] alfa = { 4.764, 6.744 };
                // int[] arr = new int[] { t.Item1, t.Item2 };
                for (int i = 0; i < alfa.Length; i++)
                {
                    alfa[i] = Math.Tan(alfa[i] * (Math.PI / 180));
                }
                // Инифиализация координат первой зоны
                p1.X = 556;
                p1.Y = 888;
                p2.X = 1420;
                p2.Y = 1488;
                //points.Add(p1);
                //points.Add(p2);
                (Profile, double) res;
                // Расчет профиля с инициализацией нового объекта класса Profile
                res = BulletFun.Calc_profile(img, p1, p2, alfa[0], file, 0, 1);
                profiles.Add(res.Item1);

                // Инифиализация координат второй зоны
                p1.X = 3340;
                p1.Y = 736;
                p2.X = 4244;
                p2.Y = 1160;
                // Расчет профиля с инициализацией нового объекта класса Profile
                res = BulletFun.Calc_profile(img, p1, p2, alfa[1], file, 0, 1);
                profiles.Add(res.Item1);
                foreach(Profile pr in profiles)
                {
                    //Console.WriteLine(string.Join("\t", pr.profile));
                    pr.profile.ForEach(i => Console.Write("{0:0.####}\t", i));
                    Console.WriteLine();
                }
                break;
            }
            // Расчет оптимального значения корелляции двух профилей путем расчета значений кросс-корелляции
            if (profiles.Count > 1)
            {
                double cross_value = 0;
                // Проверка пересечения двух зон, определяющих рассчитанные ранее профили
                int intersection = BulletFun.GetIntersection((Profile)profiles[0], (Profile)profiles[1]);
                if (intersection > 0)
                {
                    // Рассчет оптимального значения корелляции двух профилей
                    cross_value = BulletFun.Calc_cross((Profile)profiles[0], (Profile)profiles[1]);
                }
                Console.WriteLine("Cross correlation = {0}.", cross_value);
            }
        }
    }
}