// Simple test of the Java client class
//
// Keith Vertanen 4/99, updated 10/09

import java.io.*;
import java.net.*;
import java.util.Scanner; // Import the Scanner class to read text files
import java.util.ArrayList; // resizable array

public class java_client_test 
{


   public static void main( String args[] ) throws IOException
   {
	int port = 5010;
	int dataport = -1;
        String ip = "127.0.0.1";
	int rev = 1;
        
        ArrayList<String> confContents = new ArrayList<String>();
        try {
            File myObj = new File("conf.txt");
            Scanner textFileReader = new Scanner(myObj);
            while (textFileReader.hasNextLine()) {
                String data = textFileReader.nextLine();
                confContents.add(data);
            }
            textFileReader.close();
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
        
        if (confContents.size() > 0)
            port = Integer.parseInt(confContents.get(0));
        if (confContents.size() > 1)
            dataport = Integer.parseInt(confContents.get(1));
        if (confContents.size() > 2)
            ip = confContents.get(2);
        if (confContents.size() > 3)
            rev = Integer.parseInt(confContents.get(3));
        
        System.out.println("Port: " + port);
        System.out.println("Dataport: " + dataport);
        System.out.println("IP: " + ip);
        System.out.println("Reverse bytes: " + rev);
        
        Client myClient = new Client(port, dataport, ip, rev);
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
