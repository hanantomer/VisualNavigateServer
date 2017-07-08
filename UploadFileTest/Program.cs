using System;
using System.Net;

namespace UploadFileTest
{
    class Program
    {
        static void Main(string[] args)
        {
            try
            {
                WebClient wc = new WebClient();

                byte[] response = wc.UploadFile(
                    @"http://40.76.76.10/RoadView/Handler.ashx?requestType=RoadViewWithCoordinates&latitude=12.773&longitude=11.666",
                    @"C:\dev\roadview\Photos\normalized\1_1.bmp");

                string s = wc.Encoding.GetString(response);
            }
            catch (WebException e)
            {
                Console.WriteLine("This program is expected to throw WebException on successful run." +
                                    "\n\nException Message :" + e.Message);
                if (e.Status == WebExceptionStatus.ProtocolError)
                {
                    Console.WriteLine("Status Code : {0}", ((HttpWebResponse)e.Response).StatusCode);
                    Console.WriteLine("Status Description : {0}", ((HttpWebResponse)e.Response).StatusDescription);
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }
    }
}
