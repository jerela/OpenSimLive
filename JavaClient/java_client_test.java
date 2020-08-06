// Simple test of the Java client class
//
// Keith Vertanen 4/99, updated 10/09

import java.io.*;
import java.net.*;

public class java_client_test 
{


   public static void main( String args[] ) throws IOException
   {
	int port = 5010;
	int dataport = -1;
	int rev = 1;
        Client myClient = new Client(port, dataport, "127.0.0.1", rev);
        double C[];
        C = new double[6];
        
        while (C[5] < 400) // when one of the rotation values exceeds 360, we shut down the loop
        {
            System.out.println("Client receiving doubles...");
            myClient.RecvDoubles(C, 6);
        }
        
        System.out.println("Client closing connection...");
        myClient.Close();
        System.out.println("Client is finished.");
   }
}
