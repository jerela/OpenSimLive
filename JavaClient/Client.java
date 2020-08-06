// a Java socket client
//
// Keith Vertanen 11/98, updated 10/09

import java.io.*;
import java.net.*;
import java.awt.*;

public class Client 
{

   boolean VERBOSE = true;		 // turn on/off debugging output
   static int BUFFSIZE = 128000;         // how many bytes our incoming buffer can hold

   byte data[];
   byte buff[];

   int port;
   int dataport;
   String host;
   Socket sock;
   DatagramSocket recv_sock, send_sock;

   BufferedInputStream input;
   BufferedOutputStream output;
   

   // constructor, takes a port number, a machine name host, and a bit
   // that indicates whether to reverse the byte order or not

   public Client(int p, int datap, String address, int rev) throws IOException
   {
	port = p;
	dataport = datap;
	host = address;
  
        try {
	      	sock = new Socket( InetAddress.getByName(address), 
                              port );
	    	input = new BufferedInputStream(sock.getInputStream(), BUFFSIZE);
	  	output = new BufferedOutputStream(sock.getOutputStream(),BUFFSIZE);
	}
      	catch ( IOException e ) {
         	e.printStackTrace();
      	}

	if (dataport != -1)
	{
		// allocate the datagram socket
		try {
			recv_sock = new DatagramSocket(dataport);
			send_sock = new DatagramSocket();
		} 
		catch (SocketException se) {
			se.printStackTrace();
		}
	}

	// amortize the buffer allocation by just doing it once

	buff = new byte[BUFFSIZE];
	data = new byte[BUFFSIZE];

	if (VERBOSE) System.out.println("Client: opening socket to " +
			address + " on port " + port + 
			", datagrams on port = " +dataport);

	// now we want to tell the server if we want reversed bytes or not

	output.write(rev);
	output.flush();
	
	if (VERBOSE) 
		if (rev==1) 
			System.out.println("Client:  requested reversed bytes");
		else
			System.out.println("Client:  requested normal byte order");
   }

   // send a string down the socket
   public void SendString(String str) throws IOException
   {

	/* convert our string into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(str.length());

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<str.length(); i++)
		out.write((byte) str.charAt(i));

	output.write(bytestream.toByteArray(), 0, bytestream.size());
	output.flush();

 	if (VERBOSE) System.out.println("Client: sending '" + str +"'");

	RecvAck();
	SendAck();
   }

   public void SendBytes(byte vals[], int len) throws IOException
   {
	if (VERBOSE) 
	{
		System.out.print("Client: sending " + len +" bytes: ");
		for (int i=0; i<len; i++)
			System.out.print(vals[i] + " ");
	}

	output.write(vals, 0, len);
	output.flush();

 	if (VERBOSE) System.out.println("");

	RecvAck();
	SendAck();
   }

   public void SendInts(int vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Client: sending " + len +" ints: ");

	/* convert our array of ints into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*4);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeInt(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	

	output.write(bytestream.toByteArray(), 0, bytestream.size());
	output.flush();

 	if (VERBOSE) System.out.println("");

	RecvAck();
	SendAck();
   }

   public void SendFloats(float vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Client: sending " + len +" floats: ");

	/* convert our array of floats into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*4);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeFloat(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	

	output.write(bytestream.toByteArray(), 0, bytestream.size());
	output.flush();

 	if (VERBOSE) System.out.println("");

	RecvAck();
	SendAck();
   }

   public void SendDoubles(double vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Client: sending " + len +" doubles: ");

	/* convert our array of floats into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*8);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeDouble(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	

	output.write(bytestream.toByteArray(), 0, bytestream.size());
	output.flush();

	if (VERBOSE) System.out.println("");
	
	RecvAck();
	SendAck();
   }

   public void SendDatagram(byte vals[], int len) throws IOException
   {
	DatagramPacket sendPacket;

	if (VERBOSE) 
	{
		System.out.print("Client: sending datagram of " + len +" bytes: ");
		for (int i=0; i<len; i++)
			System.out.print(vals[i] + " ");
	}

	sendPacket = new DatagramPacket(vals, len,
					InetAddress.getByName(host), dataport);
	send_sock.send(sendPacket);

 	if (VERBOSE) System.out.println("");

   }

   // recv a string from the socket (terminates on terminal char)
   public String RecvString(char terminal) throws IOException
   {
	char c;
	String out;

	// would have liked to use readUTF, but it didn't seem to work
	// when talking to the c++ server

	out = new String("");

	while ((c=(char) input.read())!=terminal)
		out = out + String.valueOf(c);

 	if (VERBOSE) System.out.println("Client: recv'd '" + out +"'");

	SendAck();
	RecvAck();

	return out;
   }

  public int RecvBytes(byte val[], int maxlen) throws IOException
  {
       int i;
       int totalbytes = 0;
       int numbytes;

	if (maxlen>BUFFSIZE)
		System.out.println("Sending more bytes then will fit in buffer!");

	while (totalbytes < maxlen)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			val[i] = data[i-totalbytes];

		totalbytes += numbytes;
	}

	if (VERBOSE) 
	{
		System.out.print("Client: received " + maxlen + " bytes - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	// we now send an acknowledgement to the server to let them
	// know we've got it

	SendAck();
	RecvAck();

	return maxlen;
  }

  public int RecvInts(int val[], int maxlen) throws IOException
  {
       int i;
       int totalbytes = 0;
       int numbytes;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*4>BUFFSIZE)
		System.out.println("Sending more ints then will fit in buffer!");

	while (totalbytes < maxlen*4)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;
	}

	// now we must convert the array of bytes to an array of ints

        ByteArrayInputStream bytestream;
	DataInputStream instream;

	bytestream = new ByteArrayInputStream(buff);
	instream = new DataInputStream(bytestream);

	for (i=0; i<maxlen; i++)
		val[i] = instream.readInt();

	if (VERBOSE) 
	{
		System.out.print("Client: received " + maxlen + " ints - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	// we now send an acknowledgement to the server to let them
	// know we've got it

	SendAck();
	RecvAck();

	return maxlen;
  }

  public int RecvDoubles(double val[], int maxlen) throws IOException
  {
       int i;
       int numbytes;
       int totalbytes = 0;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*8>BUFFSIZE)
		System.out.println("Sending more doubles then will fit in buffer!");

	while (totalbytes < maxlen*8)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;
	
	}

	// now we must convert the array of bytes to an array of ints

        ByteArrayInputStream bytestream;
	DataInputStream instream;

	bytestream = new ByteArrayInputStream(buff);
	instream = new DataInputStream(bytestream);

	for (i=0; i<maxlen; i++)
		val[i] = instream.readDouble();

	if (VERBOSE) 
	{
		System.out.print("Client: received " + maxlen + " doubles - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	SendAck();
	RecvAck();

	return maxlen;
  }

  public int RecvFloats(float val[], int maxlen) throws IOException
  {
       int i;
       int numbytes;
       int totalbytes = 0;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*4>BUFFSIZE)
		System.out.println("Sending more doubles then will fit in buffer!");

	while (totalbytes < maxlen*4)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;

	}

	// now we must convert the array of bytes to an array of ints

        ByteArrayInputStream bytestream;
	DataInputStream instream;

	bytestream = new ByteArrayInputStream(buff);
	instream = new DataInputStream(bytestream);

	for (i=0; i<maxlen; i++)
		val[i] = instream.readFloat();

	if (VERBOSE) 
	{
		System.out.print("Client: received " + maxlen + " floats - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	SendAck();
	RecvAck();

	return maxlen;
  }

  public int RecvDatagram(byte val[], int maxlen) throws IOException
  {
        int i;
        int numbytes;
	DatagramPacket receivePacket;

	if (maxlen>BUFFSIZE)
		System.out.println("Sending more bytes then will fit in buffer!");


	receivePacket = new DatagramPacket(val, maxlen);
	recv_sock.receive(receivePacket);

	numbytes = receivePacket.getLength();

	if (VERBOSE) 
	{
		System.out.print("Client: received " + numbytes + " bytes - ");
		for (i=0; i<numbytes; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	return numbytes;
  }



   // shutdown the socket
   public void Close() throws IOException
   {
	sock.close();

 	if (VERBOSE) System.out.println("Client: closing socket");
   }

   // send a short ack to the server so they know we are ready for more

   private void SendAck() throws IOException
   {
	int ack;

	ack = 0;

	if (VERBOSE)
		System.out.println("Sending ack...");

	output.write(ack);
	output.flush();

   }

   // recv a short ack from the server so we know they are ready for more

   private void RecvAck() throws IOException
   {
	int ack;

	if (VERBOSE)
		System.out.println("Waiting for ack...");

	ack = (int) input.read();

	if (VERBOSE)
		System.out.println("Ack recieved.");

   }
}

