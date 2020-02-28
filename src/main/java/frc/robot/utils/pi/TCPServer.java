package frc.robot.utils.pi;

import java.io.*;
import java.net.*;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;


 
public class TCPServer {
    public volatile String data = "0 0";
    public TCPServer() {
        new Thread(conn).start();
    }
    
    Runnable conn = new Runnable() {
        public void run() {
            try {
                ServerSocket server = new ServerSocket(5010);
                long lastTime = System.nanoTime();
                while (true) {
                    Socket socket = server.accept();
                    BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    BufferedWriter out = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));
                    String str = in.readLine();
                    try {
                        if (str != null) {
                            data = str;
                            lastTime = System.nanoTime();
                        }
                        out.write("Data is " + data);
                    } catch (Exception e) {
                        System.out.println("error");
                    }
                    if (System.nanoTime() - lastTime > 5e9) {
                        break;
                    }
                    out.flush();
                    in.close();
                    out.close();
                    socket.close();
                }
            } catch (IOException e) {
            } catch (Exception e) {
            }
        }
    };

}