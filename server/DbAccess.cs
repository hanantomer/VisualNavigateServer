using Microsoft.SqlServer.Types;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data.SqlClient;
using System.Data.SqlTypes;
using System.Linq;
using System.Web;

namespace server
{
    public class DbAccess
    {
        private static DbAccess _instance = null;

        SqlConnection _conn = null;

        private static readonly log4net.ILog _log = log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        private DbAccess()
        {
            _conn = new SqlConnection();
            _conn.ConnectionString = ConfigurationManager.ConnectionStrings["ConnStr"].ConnectionString;
        }

        public static DbAccess GetInstance()
        {
            if (_instance == null)
            {
                _instance = new DbAccess();
            }
            return _instance;
        }

        public SqlConnection GetConn()
        {
            try
            {
                _conn.Open();
                return _conn;
            }
            catch (SqlException e)
            {
                _log.Error(e);
                return null;
            }
        }

        public void AddRoadViewWithCoordinates(string imagepath, double lat, double lon)
        {
            SqlGeographyBuilder builder = new SqlGeographyBuilder();

            builder.SetSrid(4326);

            builder.BeginGeography(OpenGisGeographyType.Point);
            builder.BeginFigure(lat, lon);
            builder.EndFigure();
            builder.EndGeography();

            
            SqlCommand cmd = new SqlCommand();

            SqlConnection conn = GetConn();

            try
            {
                cmd.Connection = conn;

                cmd.CommandText = 
                    "INSERT imageWithCoordinates(Coordinates, ImagePath) VALUES(@coordinates, @imagepath)";

                SqlParameter coordinatesParam = new SqlParameter();
                coordinatesParam.ParameterName = "coordinates";
                coordinatesParam.UdtTypeName = "Geography";
                coordinatesParam.Value = builder.ConstructedGeography;
                
                cmd.Parameters.Add(coordinatesParam);
                cmd.Parameters.AddWithValue("imagepath", imagepath);

                cmd.ExecuteNonQuery();
            }
            finally
            {
                conn.Close();
            }
        }
    }
}