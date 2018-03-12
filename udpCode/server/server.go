package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"net"
)

type REPORT struct {
	Head        [4]byte    //“REPT”
	Status      [2]byte    //预留
	Size        uint16     // 帧长度不会超过 256
	TimeStamp   int64      // 时间戳,注意对齐;
	QuatSensor  [4]float32 //Quaternion
	EularSensor [4]float32 // Sensor板的 Yaw, Roll, Pitch,空
	QuatMotor   [4]float32 //预留,
	EularMotor  [4]float32 // 电机的 Yaw, Roll, Pitch,空
}

func main() {
	listener, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.ParseIP("127.0.0.1"), Port: 18889})
	if err != nil {
		fmt.Println(err)
		return
	}
	var msgin REPORT
	fmt.Printf("Local: <%s> \n", listener.LocalAddr().String())

	bufin := make([]byte, 1024)
	for {
		_, remoteAddr, err := listener.ReadFromUDP(bufin)
		if err != nil {
			fmt.Printf("error during read: %s", err)
		}

		endian := binary.BigEndian

		msgin.Head[0] = bufin[0]
		msgin.Head[1] = bufin[1]
		msgin.Head[2] = bufin[2]
		msgin.Head[3] = bufin[3]

		msgin.Status[0] = bufin[1]
		msgin.Status[1] = bufin[0]

		bin2num(bufin[8:16], endian, &msgin.TimeStamp)

		bin2num(bufin[16:20], endian, &msgin.QuatSensor[0])
		bin2num(bufin[20:24], endian, &msgin.QuatSensor[1])
		bin2num(bufin[24:28], endian, &msgin.QuatSensor[2])
		bin2num(bufin[28:32], endian, &msgin.QuatSensor[3])

		bin2num(bufin[32:36], endian, &msgin.EularSensor[0])
		bin2num(bufin[36:40], endian, &msgin.EularSensor[1])
		bin2num(bufin[40:44], endian, &msgin.EularSensor[2])
		bin2num(bufin[44:48], endian, &msgin.EularSensor[3])

		bin2num(bufin[48:52], endian, &msgin.QuatMotor[0])
		bin2num(bufin[52:56], endian, &msgin.QuatMotor[1])
		bin2num(bufin[56:60], endian, &msgin.QuatMotor[2])
		bin2num(bufin[60:64], endian, &msgin.QuatMotor[3])

		bin2num(bufin[64:68], endian, &msgin.EularMotor[0])
		bin2num(bufin[68:72], endian, &msgin.EularMotor[1])
		bin2num(bufin[72:76], endian, &msgin.EularMotor[2])
		bin2num(bufin[76:80], endian, &msgin.EularMotor[3])
		fmt.Println(remoteAddr, msgin)

		_, err = listener.WriteToUDP([]byte("world"), remoteAddr)

		if err != nil {
			fmt.Printf(err.Error())
		}
		log.Println("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
	}
}
func bin2num(buf []byte, order binary.ByteOrder, pdata interface{}) {

	r := bytes.NewReader(buf)
	e := binary.Read(r, order, pdata)
	epanic(e)
}
func epanic(e error) {
	if e != nil {
		panic(e.Error())
	}
}

//package main

//import (
//	"flag"
//	"fmt"
//	"log"
//	"net"
//)

//var addr = flag.String("addr", ":10000", "udp server bing address")

//func init() {
//	log.SetFlags(log.LstdFlags | log.Lshortfile)
//	flag.Parse()
//}

//func main() {
//	//Resolving address
//	udpAddr, err := net.ResolveUDPAddr("udp", *addr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}

//	// Build listining connections
//	conn, err := net.ListenUDP("udp", udpAddr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}
//	defer conn.Close()

//	// Interacting with one client at a time
//	recvBuff := make([]byte, 1024)
//	for {
//		log.Println("Ready to receive packets!")
//		// Receiving a message
//		rn, rmAddr, err := conn.ReadFromUDP(recvBuff)
//		if err != nil {
//			log.Println("Error:", err)
//			return
//		}

//		fmt.Printf("<<< Packet received from: %s, data: %s\n", rmAddr.String(), string(recvBuff[:rn]))
//		// Sending the same message back to current client
//		_, err = conn.WriteToUDP(recvBuff[:rn], rmAddr)
//		if err != nil {
//			log.Println("Error:", err)
//			return
//		}
//		fmt.Println(">>> Sent packet to: ", rmAddr.String())
//	}
//}
