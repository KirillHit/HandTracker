namespace TCPServerListener
{
    using System;
    using System.Text;
    using System.Net;
    using System.Net.Sockets;
    using System.Threading.Tasks;

    internal class Program
    {
        private static int port = 48569;
        private static IPEndPoint ipPoint = new IPEndPoint(IPAddress.Any, port);
        private static TcpListener listener;

        private static readonly int dataBufferSize = 16384;
        private static byte[] dataBuffer = new byte[dataBufferSize];


        static void Main(string[] args)
        {
            StartupServer();
            Console.ReadKey();
        }

        /// <summary>
        /// Initial method to start server logic
        /// </summary>
        private static void StartupServer()
        {
            try
            {
                // Starting server...
                listener = new TcpListener(ipPoint);
                listener.Start();

                // Settup listening on accepting of new TCP connections
                listener.BeginAcceptTcpClient(new AsyncCallback(CallBack_SocketAccept), listener);

                Log($"Server is ready for connections on port {port}...");
            }
            catch (Exception ex)
            {
                Log(ex.Message);
            }
        }

        /// <summary>
        /// Handler for new TCP connections
        /// </summary>
        /// <param name="ar"></param>
        private static void CallBack_SocketAccept(IAsyncResult ar)
        {
            try
            {
                // Get current tcp client
                var currentTcpClient = (TcpListener)ar.AsyncState;
                currentTcpClient.BeginAcceptTcpClient(new AsyncCallback(CallBack_SocketAccept), currentTcpClient);

                // Accepted new socket connection 
                var connectedClient = currentTcpClient.EndAcceptTcpClient(ar);
                Log("Accepted new connection");

                connectedClient.Client.BeginReceive(dataBuffer, 0, 0, 0, new AsyncCallback(CallBack_DataReceived), connectedClient);
            }
            catch (Exception ex)
            {
                Log(ex.Message);
            }
        }

        /// <summary>
        /// Main data handler method
        /// </summary>
        /// <param name="ar"></param>
        private static async void CallBack_DataReceived(IAsyncResult ar)
        {
            try
            {
                // Get current TCP client for data receive
                var currentClient = (TcpClient)ar.AsyncState;

                var data = await ReadAsync(currentClient);
                if (string.IsNullOrEmpty(data))
                {
                    return;
                }

                Log($"Received: {data}");

                currentClient.Client.BeginReceive(dataBuffer, 0, 0, 0, new AsyncCallback(CallBack_DataReceived), currentClient);
            }
            catch (Exception ex)
            {
                Log($"{ex.Message}");
            }
        }

        private static async Task<string> ReadAsync(TcpClient client)
        {
            var stream = client.GetStream();
            var stringBuilder = new StringBuilder();

            int totalReaded = 0;
            while (stream.DataAvailable)
            {
                var actuallyRead = await stream.ReadAsync(dataBuffer, totalReaded, dataBufferSize);
                if (actuallyRead == 0) break;

                stringBuilder.Append(Encoding.UTF8.GetString(dataBuffer), 0, actuallyRead);
                totalReaded += actuallyRead;
            }

            await stream.FlushAsync();
            return stringBuilder.ToString();
        }

        private static void Log(string message) => Console.WriteLine(message);
    }
}
