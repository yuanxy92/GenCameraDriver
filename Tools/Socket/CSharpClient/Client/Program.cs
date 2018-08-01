using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;

namespace Client
{
    class Program
    {
        private static List<string> ReadServerAddress(string configname)
        {
            var serverAddress = new List<string>();
            // Read the file and display it line by line.  
            int counter = 0;
            string line;
            System.IO.StreamReader file = new System.IO.StreamReader(@configname);
            while ((line = file.ReadLine()) != null)
            {
                System.Console.WriteLine(line);
                serverAddress.Add(line);
                counter++;
            }
            file.Close();
            System.Console.WriteLine("There were {0} servers.", counter);
            return serverAddress;
        }

        private static void SendCallback(IAsyncResult ar)
        {
            try
            {
                // Retrieve the socket from the state object.  
                Socket client = (Socket)ar.AsyncState;
                // Complete sending the data to the remote device.  
                int bytesSent = client.EndSend(ar);
                Console.WriteLine("Sent action to {0}, total {1} bytes.", 
                    client.RemoteEndPoint.ToString(), bytesSent);
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }

        public static int StartClient(List<string> servers)
        {
            // Data buffer for incoming data.  
            byte[] bytes = new byte[1024];
            // Connect to a remote device.  
            try
            {
                // create socket
                List<IPAddress> ipAddress = new List<IPAddress>();
                List<IPEndPoint> remoteEP = new List<IPEndPoint>();
                //foreach(string ipFull in servers)
                for (int ind = 0; ind < servers.Count; ind ++)
                {
                    var ipFull = servers[ind];
                    string[] address = ipFull.Split(':');
                    string ip = address[0];
                    int port = int.Parse(address[1]);
                    ipAddress.Add(IPAddress.Parse(ip));
                    remoteEP.Add(new IPEndPoint(ipAddress[ind], port));
                }
                try
                {
                    // Create TCP/IP socket and connect
                    var senders = new List<Socket>();
                    for (int ind = 0; ind < servers.Count; ind++)
                    {
                        senders.Add(new Socket(ipAddress[ind].AddressFamily,
                            SocketType.Stream, ProtocolType.Tcp));
                        senders[ind].Connect(remoteEP[ind]);
                        Console.WriteLine("Socket connected to {0}",
                        senders[ind].RemoteEndPoint.ToString());
                    }
                    // send message
                    // Connect the socket to the remote endpoint. Catch any errors.  
                    // Encode the data string into a byte array.  
                    byte[] msg = Encoding.ASCII.GetBytes("Action");
                    // async send
                    for (int ind = 0; ind < servers.Count; ind++)
                    {
                        // Begin sending the data to the remote device.  
                        senders[ind].BeginSend(msg, 0, msg.Length, 0,
                            new AsyncCallback(SendCallback), senders[ind]);
                    } 

                }
                catch (ArgumentNullException ane)
                {
                    Console.WriteLine("ArgumentNullException : {0}", ane.ToString());
                }
                catch (SocketException se)
                {
                    Console.WriteLine("SocketException : {0}", se.ToString());
                }
                catch (Exception e)
                {
                    Console.WriteLine("Unexpected exception : {0}", e.ToString());
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }

            return 0;
        }

        static int Main(string[] args)
        {
            var serverAddress = ReadServerAddress("server.txt");
            StartClient(serverAddress);
            return 0;
        }
    }
}
