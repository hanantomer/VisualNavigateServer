using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Web;

namespace server
{
    /// <summary>
    /// Summary description for Handler
    /// </summary>
    public class Handler : IHttpHandler
    {
        private static readonly log4net.ILog _log =
            log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        private static readonly string FilesDirectory = @"C:\RoadViewImages";

        public void ProcessRequest(HttpContext context)
        {
            try
            {
                _log.Info("accepting request");

                string requestType = context.Request.Params["requestType"].ToString();

                _log.Info("request type:" + requestType);

                if (requestType == "log")
                {
                    string logType = context.Request.Params["logType"].ToString();

                    _log.Info("log type:" + logType);

                    string message = context.Request.Params["message"].ToString();

                    if (logType == "info")
                    {
                        _log.Info(message);
                    }
                    else if (logType == "error")
                    {
                        _log.Error(message);
                    }
                    else
                    {
                        _log.Debug(message);
                    }

                }
                if (requestType == "RoadViewWithCoordinates")
                {
                    string imagePath = SaveImage(context.Request.Files[0]);

                    _log.Info("imagePath:" + imagePath);

                    double latitude = Double.Parse(context.Request.Params["latitude"]);

                    _log.Info("latitude:" + latitude);

                    double longitude = Double.Parse(context.Request.Params["longitude"]);

                    _log.Info("longitude:" + longitude);

                    DbAccess.GetInstance().AddRoadViewWithCoordinates(imagePath, latitude, longitude);
                }
            }
            catch(Exception e)
            {
                _log.Error(e);
                context.Response.Write(e.ToString());
            }
        }
        
        private string SaveImage(HttpPostedFile httpPostedFile)
        {
            string fileName = FilesDirectory + "\\" + DateTime.Now.Ticks + ".jpg";

            httpPostedFile.SaveAs(fileName);

            return fileName;
        }

        public bool IsReusable
        {
            get
            {
                return false;
            }
        }
    }
}