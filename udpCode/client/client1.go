package main

import (
	"bytes"
	"encoding/binary"
	"flag"
	"fmt"
	//	"fmt"
	"log"
	"net"
	"time"

	"math"

	"github.com/tarm/serial"
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

const PI = 3.1415926

var raddr = flag.String("raddr", "127.0.0.1:18889", "remote server address")
var outmsg REPORT

func init() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	flag.Parse()
}

func main() {
	sensortest()
	time.Sleep(time.Second * 12)
	// Resolving Address
	remoteAddr, err := net.ResolveUDPAddr("udp", *raddr)
	if err != nil {
		log.Fatalln("Error: ", err)
	}
	// Make a connection
	tmpAddr := &net.UDPAddr{
		IP:   net.ParseIP("127.0.0.1"),
		Port: 0,
	}

	conn, err := net.DialUDP("udp", tmpAddr, remoteAddr)
	// Exit if some error occured
	if err != nil {
		log.Fatalln("Error: ", err)
	}
	endian := binary.BigEndian
	defer conn.Close()
	go sensortest()

	for { //

		//		outbuf := make([]byte, binary.Size(outmsg), binary.Size(outmsg))
		//fmt.Println(len(outbuf))
		outmsg.TimeStamp = int64(int(time.Now().Weekday())*24*3600+
			time.Now().Hour()*3600+time.Now().Minute()*60+
			time.Now().Second()*1000) + int64(time.Now().Nanosecond()/1000000)
		outbuf := make([]byte, binary.Size(outmsg), binary.Size(outmsg))
		buf0 := new(bytes.Buffer)
		buf1 := new(bytes.Buffer)
		buf2 := new(bytes.Buffer)
		buf3 := new(bytes.Buffer)
		buf4 := new(bytes.Buffer)
		buf5 := new(bytes.Buffer)
		buf6 := new(bytes.Buffer)
		buf7 := new(bytes.Buffer)
		buf8 := new(bytes.Buffer)
		buf9 := new(bytes.Buffer)
		buf10 := new(bytes.Buffer)
		buf11 := new(bytes.Buffer)
		buf12 := new(bytes.Buffer)
		buf13 := new(bytes.Buffer)
		buf14 := new(bytes.Buffer)
		buf15 := new(bytes.Buffer)
		buf16 := new(bytes.Buffer)
		//头部
		outbuf[0] = byte('R')
		outbuf[1] = byte('E')
		outbuf[2] = byte('P')
		outbuf[3] = byte('T')
		//预留
		outbuf[4] = 0
		outbuf[5] = 0
		//贞长
		outmsg.Size = 80
		binary.Write(buf0, endian, outmsg.Size) //
		copy(outbuf[6:], buf0.Bytes())
		//四元素
		binary.Write(buf1, endian, outmsg.QuatSensor[0]) //
		copy(outbuf[16:], buf1.Bytes())
		binary.Write(buf2, endian, outmsg.QuatSensor[1]) //
		copy(outbuf[20:], buf2.Bytes())
		binary.Write(buf3, endian, outmsg.QuatSensor[2]) //
		copy(outbuf[24:], buf3.Bytes())
		binary.Write(buf4, endian, outmsg.QuatSensor[3]) //
		copy(outbuf[28:], buf4.Bytes())

		binary.Write(buf5, endian, outmsg.EularSensor[0]) //
		copy(outbuf[32:], buf5.Bytes())
		binary.Write(buf6, endian, outmsg.EularSensor[1]) //
		copy(outbuf[36:], buf6.Bytes())
		binary.Write(buf7, endian, outmsg.EularSensor[2]) //
		copy(outbuf[40:], buf7.Bytes())
		binary.Write(buf8, endian, outmsg.EularSensor[3]) //
		copy(outbuf[44:], buf8.Bytes())

		binary.Write(buf9, endian, outmsg.QuatMotor[0]) //
		copy(outbuf[48:], buf9.Bytes())
		binary.Write(buf10, endian, outmsg.QuatMotor[1]) //
		copy(outbuf[52:], buf10.Bytes())
		binary.Write(buf11, endian, outmsg.QuatMotor[2]) //
		copy(outbuf[56:], buf11.Bytes())
		binary.Write(buf12, endian, outmsg.QuatMotor[3]) //
		copy(outbuf[60:], buf12.Bytes())

		binary.Write(buf13, endian, outmsg.EularMotor[0]) //
		copy(outbuf[64:], buf13.Bytes())
		binary.Write(buf14, endian, outmsg.EularMotor[1]) //
		copy(outbuf[68:], buf14.Bytes())
		binary.Write(buf15, endian, outmsg.EularMotor[2]) //
		copy(outbuf[72:], buf15.Bytes())
		binary.Write(buf16, endian, outmsg.EularMotor[3]) //
		copy(outbuf[76:], buf16.Bytes())
		// write a message to server
		_, err = conn.Write([]byte(outbuf)) //_, err = listener.WriteToUDP([]byte("world"), remoteAddr)
		if err != nil {
			log.Println(err)
		} /*else {
			//fmt.Println(">>> Packet sent to: ", *raddr)
		}*/

		// Receive response from server
		//buf := make([]byte, 1024)
		//rn, rmAddr, err := conn.ReadFromUDP(buf)
		//		if err != nil {
		//			log.Println(err)
		//		} else {
		//			fmt.Printf("<<<  %d bytes received from: %v, data: %s\n", rn, rmAddr, string(buf[:rn]))
		//		}
		time.Sleep(time.Millisecond * 20)
		//fmt.Println(outmsg)
	}

}

var Quat []float64

func sensortest() {
	//打开串口
	Quat = make([]float64, 4)
	c := &serial.Config{Name: "COM3", Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}
	n, err := s.Write([]byte("test"))
	if err != nil {
		log.Fatal(err)
	}
	buf := make([]byte, 256)
	go func() {
		for {

			n, err = s.Read(buf) //数据接收存储于buf中
			if err != nil {
				log.Fatal(err)
			}
			log.Println("共采集：", n, "个数")
			log.Printf("共采集：%#v\n", buf[:n])
			time.Sleep(time.Millisecond * 10)
		}
	}()
	time.Sleep(time.Millisecond * 100)
	go func() {

		Q := make([]byte, 9)

		for {
			//从
			var i int16 = 0
			var ok bool = false
			n, err = s.Read(buf)
			for !ok {
				ok = ((buf[i] == 0x55) && (buf[i+1] == 0x59)) || i >= int16(n)
				i++
			}
			time.Sleep(time.Millisecond * 20)
			//log.Printf("%#v\n", buf[:n])
			//数据分类存储
			switch buf[i] {
			case 0x59:
				i++
				Q[0], Q[1], Q[2], Q[3], Q[4], Q[5], Q[6], Q[7], Q[8] = buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7], buf[i+8]
			}
			fmt.Println("传输无误", buf[i+10], buf[i+9])
			Quat[0] = float64(int16(Q[1])<<8|int16(Q[0])) / 32768.0 //单位是g//float64((int16(high_byte)<<8)|int16(low_byte)) * 160.0 / 32768.0
			Quat[1] = float64(int16(Q[3])<<8|int16(Q[2])) / 32768.0 //单位是g
			Quat[2] = float64(int16(Q[5])<<8|int16(Q[4])) / 32768.0 //单位是g
			Quat[3] = float64(int16(Q[7])<<8|int16(Q[6])) / 32768.0 //单位是g
			fmt.Printf("四元素 %#v\n", outmsg.QuatSensor)
		}
	}()

	go func() {

		Eular := make([]byte, 9)
		for {

			var ok bool = false
			var i int16 = 0
			for !ok {
				ok = ((buf[i] == 0x55) && (buf[i+1] == 0x53)) || i >= int16(n)
				i++
			}
			time.Sleep(time.Millisecond * 20)
			//log.Printf("%#v\n", buf[:n])

			switch buf[i] {
			case 0x53:
				i++
				Eular[0], Eular[1], Eular[2], Eular[3], Eular[4], Eular[5], Eular[6], Eular[7], Eular[8] =
					buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7], buf[i+8]
				outmsg.EularSensor[2] = float32(int16(Eular[1])<<8|int16(Eular[0])) * 180.0 / 32768.0 //单位是g//float64((int16(high_byte)<<8)|int16(low_byte)) * 160.0 / 32768.0
				outmsg.EularSensor[1] = float32(int16(Eular[3])<<8|int16(Eular[2])) * 180.0 / 32768.0 //单位是g
				outmsg.EularSensor[0] = float32(int16(Eular[5])<<8|int16(Eular[4])) * 180.0 / 32768.0 //单位是g
				//fmt.Printf("欧拉角 %#v\n", outmsg.EularSensor)
				EularToQuat(outmsg.EularSensor)
				//log.Println(outmsg.QuatSensor)
				//log.Println(bb)
			}
		}
	}()
}

//输入欧拉角Rx,Ry,Rz.输出Qw,Qx,Qy,Qz
func EularToQuat(Eular [4]float32) []float32 {
	Quat := make([]float32, 4)
	Quat[0] = float32(math.Cos(float64(Eular[0]*PI/180.0))*math.Cos(float64(Eular[1]*PI/180.0))*math.Cos(float64(Eular[2]*PI/180.0)) + math.Sin(float64(Eular[0]*PI/180.0))*math.Sin(float64(Eular[1]*PI/180.0))*math.Sin(float64(Eular[2])*PI/180.0))
	Quat[1] = float32(math.Sin(float64(Eular[0]*PI/180.0))*math.Cos(float64(Eular[1]*PI/180.0))*math.Cos(float64(Eular[2]*PI/180.0)) - math.Cos(float64(Eular[0]*PI/180.0))*math.Sin(float64(Eular[1]*PI/180.0))*math.Sin(float64(Eular[2])*PI/180.0))
	Quat[2] = float32(math.Cos(float64(Eular[0]*PI/180.0))*math.Sin(float64(Eular[1]*PI/180.0))*math.Cos(float64(Eular[2]*PI/180.0)) + math.Sin(float64(Eular[0]*PI/180.0))*math.Cos(float64(Eular[1]*PI/180.0))*math.Sin(float64(Eular[2])*PI/180.0))
	Quat[3] = float32(math.Cos(float64(Eular[0]*PI/180.0))*math.Cos(float64(Eular[1]*PI/180.0))*math.Sin(float64(Eular[2]*PI/180.0)) - math.Sin(float64(Eular[0]*PI/180.0))*math.Sin(float64(Eular[1]*PI/180.0))*math.Cos(float64(Eular[2])*PI/180.0))

	outmsg.QuatSensor[0] = Quat[0]
	outmsg.QuatSensor[1] = Quat[1]
	outmsg.QuatSensor[2] = Quat[2]
	outmsg.QuatSensor[3] = Quat[3]
	return Quat
}
