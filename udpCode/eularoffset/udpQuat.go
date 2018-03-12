/*直接调用这个方法QuatReport()就开始采集数据并发送UDP报文。
StopReport()停止上报UDP报文
注意：sensor数据上报的串口写死：COM3,115200          */
package reportQuat

import (
	"bytes"
	"encoding/binary"
	"flag"
	"fmt"
	//. "fmt"
	"os"
	//	"fmt"

	"log"
	"math"
	. "math"
	"net"
	//	"os"
	"time"

	//	"irock.ren/Aircraft/xmotion"
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

const speed = 22
const PI = 3.1415926

var raddr = flag.String("raddr", "127.0.0.1:18889", "remote server address")
var eular [4]byte
var beforPoint [4]float64
var nowPoint [4]float64

var yaw_, roll_, pitch_, high_ [speed + 1]float64
var yaw_filter, roll_filter, pitch_filter, high_filter [speed + 1]float64
var yaw_out, roll_out, pitch_out, high_out [speed + 1]float64

var stopTime int
var index int //存储当前发送角度数组的index
var outmsg REPORT
var enable bool = true   //to end this package
var update bool = false  //刷新一次就变TRUE，使用这次刷新后就变FALSE
var diraction [4]float64 //反应当前旋转方向，主要用于标记360旋转的地方

//循环调用本程序向包刷新座椅的欧拉角,再结合当前发送的欧拉角产生新的一组序列
func UdpMoterEular(yaw, roll, pitch, high float64) {
	nowPoint[0], nowPoint[1], nowPoint[2], nowPoint[3] = yaw, roll, pitch, high
	inputfilter()
	update = true
	beforPoint[0], beforPoint[1], beforPoint[2], beforPoint[3] = yaw, roll, pitch, high
}

var index_startFilter int //开始输入滤波时，输出的index
var index_stopFilter int  //结束输入滤波时，输出的index
var n int                 //滤波时产生的index移动
func inputfilter() {

	var stepYaw, stepRoll, stepPitch float64
	index_startFilter = index
	if index+n < speed {
		index = index + n
	} else {
		index = speed - 1
	}
	gamNowYaw := yaw_filter[index]
	gamNowRoll := roll_filter[index]
	gamNowPitch := pitch_filter[index]
	gamNowHigt := high_filter[index]

	stepYaw = (nowPoint[0] - gamNowYaw) / speed
	stepRoll = (nowPoint[1] - gamNowRoll) / speed
	stepPitch = (nowPoint[2] - gamNowPitch) / speed
	stepHigh := (nowPoint[3] - gamNowHigt) / speed
	for i := 0; i < speed; i++ {
		yaw_filter[i] = gamNowYaw + stepYaw*float64(i)
		roll_filter[i] = gamNowRoll + stepRoll*float64(i)
		pitch_filter[i] = gamNowPitch + stepPitch*float64(i)
		high_filter[i] = gamNowHigt + stepHigh*float64(i)
	}
	//穿越360°时需要，规划一步的大小和方向。
	for i := 0; i <= speed; i++ {
		yaw_[i] = yaw_filter[i]
		for yaw_[i] > 360 {
			yaw_[i] -= 360
		}
		for yaw_[i] < 0 {
			yaw_[i] += 360
		}
		roll_[i] = roll_filter[i]
		for roll_[i] > 360 {
			roll_[i] -= 360
		}
		for roll_[i] < 0 {
			roll_[i] += 360
		}
		pitch_[i] = pitch_filter[i]
		for pitch_[i] > 360 {
			pitch_[i] -= 360
		}
		for pitch_[i] < 0 {
			pitch_[i] += 360
		}
		high_[i] = high_filter[i]
	}

}
func dataFromfilter() {
	index_stopFilter = index
	n = index_stopFilter - index_startFilter + 1

	for i := 0; i <= (speed - n); i++ {
		yaw_out[i] = yaw_[n+i]
		roll_out[i] = roll_[n+i]
		pitch_out[i] = pitch_[n+i]
		high_out[i] = high_[n+i]
	}
	for i := (speed - n); i < speed; i++ {
		yaw_out[i] = yaw_[i]
		roll_out[i] = roll_[i]
		pitch_out[i] = pitch_[i]
		high_out[i] = high_[i]
	}
}
func UdpEularMotorUseTheEular() {
	remoteAddr, err := net.ResolveUDPAddr("udp", *raddr)
	if err != nil {
		log.Fatalln("Error: ", err)
	}
	tmpAddr := &net.UDPAddr{
		IP:   net.ParseIP("127.0.0.1"),
		Port: 0,
	}
	conn, err := net.DialUDP("udp", tmpAddr, remoteAddr)
	if err != nil {
		log.Fatalln("Error: ", err)
	}
	endian := binary.BigEndian
	defer conn.Close()
	for {
		enable = true
		if update == true {
			dataFromfilter()
		}
		update = false
		for i := 0; i < speed; i++ {
			timer := time.NewTimer(time.Millisecond * 10)
			outmsg.EularMotor[0] = float32(yaw_out[i]) //+ float32(math.Asin(math.Sin((roll_out[i])*PI/180)*math.Sin(pitch_out[i]*PI/180))/PI*180)
			outmsg.EularMotor[1] = float32(roll_out[i])
			outmsg.EularMotor[2] = float32(pitch_out[i]) // (math.Acos(math.Cos((roll_out[i]+90.0)*PI/180)*math.Cos(pitch_out[i]*PI/180)) / PI * 180)
			//Quat := make([]float32, 4)
			//Quat[0]是QX,Quat[1]是QY,Quat[2]是QZ,Quat[3]是QW
			angle := Angle{Xp: float64(outmsg.EularMotor[2]),
				Yy: float64(outmsg.EularMotor[0]), Zr: float64(outmsg.EularMotor[1])}
			quat := Angle2Quat(angle)
			quat.X *= -1 // right hand->left hand
			quat.Y *= -1
			quat.Z *= -1
			/*			Quat[0] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
						Quat[1] = float32(math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
						Quat[2] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
						Quat[3] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2])*PI/360.0))
			*/
			outmsg.QuatMotor[0] = float32(quat.W) // Quat[3]
			outmsg.QuatMotor[1] = float32(quat.X) // Quat[0]
			outmsg.QuatMotor[2] = float32(quat.Y) // Quat[1]
			outmsg.QuatMotor[3] = float32(quat.Z) // Quat[2]
			//og.Println(i, outmsg.EularMotor, outmsg.QuatMotor)
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
			//outbuf[5] = 3//这个数等于3时直接上报欧拉角
			outbuf[5] = 1 //1时上报四元数数据
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
			log.Printf("%#v", outmsg.EularMotor)
			_, err = conn.Write([]byte(outbuf)) //_, err = listener.WriteToUDP([]byte("world"), remoteAddr)
			if err != nil {
				log.Println(err)
			}

			index = i
			//如果数据更新了就要进入新的循环
			if update == true {
				<-timer.C
				break
			}

			<-timer.C
			if enable == false {
				<-timer.C
				break
			}
		}
		if update == false {
			log.Println("数据刷新过慢")
			//			for !update { //由于数组已经刷完，所以在这里忙等
			//			}
		}
		if enable == false {
			return
		}
	}
}
func TEST() {
	go UdpEularMotorUseTheEular()
	var Yaw float64 = 10
	file_out, _ := os.Create("输入输出测试0" + ".txt")
	go func() {
		for i := 0; ; i++ {
			timer := time.NewTimer(time.Millisecond * 10)
			str := fmt.Sprintf("%d   %3.3f   %3.3f", i, Yaw, outmsg.EularMotor[1])
			file_out.WriteString(str + "\n")
			<-timer.C
		}
	}()
	for i := 0; ; i++ {
		time.Sleep(time.Millisecond * 100)
		Yaw = 180 + 180*math.Sin(float64(i)*PI/500)
		UdpMoterEular(0, 90, Yaw, 0.0)

		log.Println(Yaw)

	}

}

type Quat struct {
	X, Y, Z, W float64 //Go math package don't have float32 functions.
}

//Angle is not strictly Euler-Angle,
//mainly for Motor's abusolute angle.
type Angle struct {
	Xp, Yy, Zr float64 // X-ptich, Y-yaw, Z-roll
}

/*
func main() {

	Printf("abs angle to quat&euler test.\n")
	//angle := make([]float64, 3, 3)
	//var quat []float64

	//angle[0] = 0   //X, Pitch;;;[180,0,0]=={1,0,0,0};;;[-180,0,0]==[0,180,180]=[0,-180,-180]={-1,0,0,0};;;
	//angle[1] = 180 //Y, Yaw,
	//angle[2] = 180 //Z, Roll,

	angle := Angle{Xp: 0, Yy: 180, Zr: 180}

	quat := Angle2Quat(angle)

	Printf("XYZ(Pitch,Yaw,Roll): %v\n", angle)
	Printf("XYZW: %v\n", quat)

	////////////////////////////////////////////////////////////////////////////

	aOculus := Angle{Xp: 20, Yy: 20, Zr: 20}
	aSeat := Angle{Xp: 20, Yy: 20, Zr: 20}

	qOculus := Angle2Quat(aOculus)
	qSeat := Angle2Quat(aSeat)
	qHead := QuatDiff(qOculus, qSeat)

	aHead := Quat2Angle(qHead)

	Printf("qHead,aHead: %#v, %#v\n", qHead, aHead)

}*/

func Angle2Quat(a Angle) Quat {

	q := Quat{}
	PI := Pi //math.Pi

	a.Xp = a.Xp * PI / 360.0
	a.Yy = a.Yy * PI / 360.0
	a.Zr = a.Zr * PI / 360.0
	//W,X,Y,Z
	q.W = Cos(a.Xp)*Cos(a.Yy)*Cos(a.Zr) + Sin(a.Xp)*Sin(a.Yy)*Sin(a.Zr)
	q.X = Sin(a.Xp)*Cos(a.Yy)*Cos(a.Zr) - Cos(a.Xp)*Sin(a.Yy)*Sin(a.Zr)
	q.Y = Cos(a.Xp)*Sin(a.Yy)*Cos(a.Zr) + Sin(a.Xp)*Cos(a.Yy)*Sin(a.Zr)
	q.Z = Cos(a.Xp)*Cos(a.Yy)*Sin(a.Zr) - Sin(a.Xp)*Sin(a.Yy)*Cos(a.Zr)

	return q
}

func Quat2Angle(q Quat) Angle {

	a := Angle{}
	return a

}

//input : X(Pitch),Y(Yaw),Z(Roll)
//output: X,Y,Z,W ;;; Oculus default use this;
//func Angle2Quat(angle []float64) []float64 {

//	quat := make([]float64, 4, 4)
//	Quat := make([]float64, 4, 4)

//	PI := math.Pi
//	//W,X,Y,Z
//	quat[0] = math.Cos(float64(angle[0]*PI/360.0))*math.Cos(float64(angle[1]*PI/360.0))*math.Cos(float64(angle[2]*PI/360.0)) + math.Sin(float64(angle[0]*PI/360.0))*math.Sin(float64(angle[1]*PI/360.0))*math.Sin(float64(angle[2])*PI/360.0)
//	quat[1] = math.Sin(float64(angle[0]*PI/360.0))*math.Cos(float64(angle[1]*PI/360.0))*math.Cos(float64(angle[2]*PI/360.0)) - math.Cos(float64(angle[0]*PI/360.0))*math.Sin(float64(angle[1]*PI/360.0))*math.Sin(float64(angle[2])*PI/360.0)
//	quat[2] = math.Cos(float64(angle[0]*PI/360.0))*math.Sin(float64(angle[1]*PI/360.0))*math.Cos(float64(angle[2]*PI/360.0)) + math.Sin(float64(angle[0]*PI/360.0))*math.Cos(float64(angle[1]*PI/360.0))*math.Sin(float64(angle[2])*PI/360.0)
//	quat[3] = math.Cos(float64(angle[0]*PI/360.0))*math.Cos(float64(angle[1]*PI/360.0))*math.Sin(float64(angle[2]*PI/360.0)) - math.Sin(float64(angle[0]*PI/360.0))*math.Sin(float64(angle[1]*PI/360.0))*math.Cos(float64(angle[2])*PI/360.0)

//	Quat[0] = quat[1] //X
//	Quat[1] = quat[2] //Y
//	Quat[2] = quat[3] //Z
//	Quat[3] = quat[0] //W

//	return Quat
//}

func AngleOff(q1, q2 []float64) float64 {

	return 2 * Acos(Abs(q1[0]*q2[0]+q1[1]*q2[1]+q1[2]*q2[2]+q1[3]*q2[3])) / Pi * 180

}

//LeftMulti: qFinal = qDiff * q
func (q Quat) Rotate(qDiff Quat) Quat {

	return QuatMulti(qDiff, q)

}

//unit quat's inverse is the equal to conjugate
func (q Quat) Inverse() Quat {

	return Quat{X: -q.X, Y: -q.Y, Z: -q.Z, W: q.W}

}

//qFinal左乘qInit的共轭
func QuatDiff(qFinal, qInit Quat) Quat {
	return QuatMulti(qInit.Inverse(), qFinal)

}

//不满足交换律
func QuatMulti(qL, qR Quat) Quat {

	return Quat{
		X: qL.W*qR.X + qL.X*qR.W + qL.Y*qR.Z - qL.Z*qR.Y,
		Y: qL.W*qR.Y - qL.X*qR.Z + qL.Y*qR.W + qL.Z*qR.X,
		Z: qL.W*qR.Z + qL.X*qR.Y - qL.Y*qR.X + qL.Z*qR.W,
		W: qL.W*qR.W - qL.X*qR.X - qL.Y*qR.Y - qL.Z*qR.Z,
		//     w * b.x + x * b.w + y * b.z - z * b.y,
		//     w * b.y - x * b.z + y * b.w + z * b.x,
		//     w * b.z + x * b.y - y * b.x + z * b.w,
		//     w * b.w - x * b.x - y * b.y - z * b.z);
	}

}

/*//座椅的欧拉角。
//RX=座椅+电动缸+补偿

// 座椅=圈数/传动比
// 电动缸=圈数->高度->角度

//RY=大臂+补偿
//RZ=大转盘+补偿
//再将欧拉角转化为电机四元素。*/
/*NX大转盘、Ny大臂、Nz座椅、Nw电动缸
由于NX,NY,NZ实际返回来的是50ms一次，
而要求刷新的是。用速度不变原则推断当前位置*/

/*向oclus发送电机位置产生四元素UDP:   首先通过go UDPEularMotor()向游戏中发送四元素数据
再在刷新下发位置的循环中调用FeedBackfilter(r0, r1, r2, r3)每次循环都会调用这个函数以刷新位置数据*/
//func FeedBackfilter(r0, r1, r2, r3 float64) {

//	step0 := (r0 - Nx[0]) / 5
//	step1 := (r1 - Ny[0]) / 5
//	step2 := (r2 - Nz[0]) / 5
//	step3 := (r3 - Nw[0]) / 5
//	for i := 0; i < 50; i++ {
//		Nx[i] = r0 + float64(i)*step0
//		Ny[i] = r1 + float64(i)*step1
//		Nz[i] = r2 + float64(i)*step2
//		Nw[i] = r3 + float64(i)*step3
//	}
//}
//func UdpEularMotor() {

//	remoteAddr, err := net.ResolveUDPAddr("udp", *raddr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}
//	tmpAddr := &net.UDPAddr{
//		IP:   net.ParseIP("127.0.0.1"),
//		Port: 0,
//	}
//	conn, err := net.DialUDP("udp", tmpAddr, remoteAddr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}
//	endian := binary.BigEndian
//	defer conn.Close()
//	for {
//		state = false
//		enable = true
//		for i := 0; i < 10; i++ {
//			timer := time.NewTimer(time.Millisecond * 10)
//			Rchai := Nz[i] / 50 * 360 //°

//			RH := math.Acos((72*72+72*72-(Nw[i]/5.17)*(Nw[i]/5.17))/(2*72*72)) / PI * 360 //
//			outmsg.EularMotor[1] = float32(Nx[i] / 40 * 360)                              //+补偿
//			for outmsg.EularMotor[1] > 360 {
//				outmsg.EularMotor[1] = (outmsg.EularMotor[1] - 360)
//			}
//			for outmsg.EularMotor[1] < -5 {
//				outmsg.EularMotor[1] = outmsg.EularMotor[1] + 360
//			}
//			if outmsg.EularMotor[1] > 180 {
//				outmsg.EularMotor[1] = outmsg.EularMotor[1] - 180
//			}
//			outmsg.EularMotor[2] = float32(Rchai+RH) + 180 //+补偿
//			for outmsg.EularMotor[2] > 360 {
//				outmsg.EularMotor[2] = outmsg.EularMotor[2] - 360
//			}
//			for outmsg.EularMotor[2] < 0 {
//				outmsg.EularMotor[2] = outmsg.EularMotor[2] + 360
//			}

//			outmsg.EularMotor[0] = float32(Ny[i] / 60 * 360) //+补偿
//			for outmsg.EularMotor[0] > 360 {
//				outmsg.EularMotor[0] = outmsg.EularMotor[0] - 360
//			}
//			for outmsg.EularMotor[0] < 0 {
//				outmsg.EularMotor[0] = outmsg.EularMotor[0] + 360
//			}

//			Quat := make([]float32, 4)
//			Quat[0] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[1] = float32(math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[2] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[3] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2])*PI/360.0))
//			outmsg.QuatMotor[0] = Quat[0]
//			outmsg.QuatMotor[1] = Quat[1]
//			outmsg.QuatMotor[2] = Quat[2]
//			outmsg.QuatMotor[3] = Quat[3]
//			//og.Println(i, outmsg.EularMotor, outmsg.QuatMotor)
//			outmsg.TimeStamp = int64(int(time.Now().Weekday())*24*3600+
//				time.Now().Hour()*3600+time.Now().Minute()*60+
//				time.Now().Second()*1000) + int64(time.Now().Nanosecond()/1000000)
//			outbuf := make([]byte, binary.Size(outmsg), binary.Size(outmsg))
//			buf0 := new(bytes.Buffer)
//			buf1 := new(bytes.Buffer)
//			buf2 := new(bytes.Buffer)
//			buf3 := new(bytes.Buffer)
//			buf4 := new(bytes.Buffer)
//			buf5 := new(bytes.Buffer)
//			buf6 := new(bytes.Buffer)
//			buf7 := new(bytes.Buffer)
//			buf8 := new(bytes.Buffer)
//			buf9 := new(bytes.Buffer)
//			buf10 := new(bytes.Buffer)
//			buf11 := new(bytes.Buffer)
//			buf12 := new(bytes.Buffer)
//			buf13 := new(bytes.Buffer)
//			buf14 := new(bytes.Buffer)
//			buf15 := new(bytes.Buffer)
//			buf16 := new(bytes.Buffer)
//			//头部
//			outbuf[0] = byte('R')
//			outbuf[1] = byte('E')
//			outbuf[2] = byte('P')
//			outbuf[3] = byte('T')
//			//预留
//			outbuf[4] = 0
//			outbuf[5] = 1
//			//贞长
//			outmsg.Size = 80

//			binary.Write(buf0, endian, outmsg.Size) //
//			copy(outbuf[6:], buf0.Bytes())
//			//四元素
//			binary.Write(buf1, endian, outmsg.QuatSensor[0]) //
//			copy(outbuf[16:], buf1.Bytes())
//			binary.Write(buf2, endian, outmsg.QuatSensor[1]) //
//			copy(outbuf[20:], buf2.Bytes())
//			binary.Write(buf3, endian, outmsg.QuatSensor[2]) //
//			copy(outbuf[24:], buf3.Bytes())
//			binary.Write(buf4, endian, outmsg.QuatSensor[3]) //
//			copy(outbuf[28:], buf4.Bytes())

//			binary.Write(buf5, endian, outmsg.EularSensor[0]) //
//			copy(outbuf[32:], buf5.Bytes())
//			binary.Write(buf6, endian, outmsg.EularSensor[1]) //
//			copy(outbuf[36:], buf6.Bytes())
//			binary.Write(buf7, endian, outmsg.EularSensor[2]) //
//			copy(outbuf[40:], buf7.Bytes())
//			binary.Write(buf8, endian, outmsg.EularSensor[3]) //
//			copy(outbuf[44:], buf8.Bytes())

//			binary.Write(buf9, endian, outmsg.QuatMotor[0]) //
//			copy(outbuf[48:], buf9.Bytes())
//			binary.Write(buf10, endian, outmsg.QuatMotor[1]) //
//			copy(outbuf[52:], buf10.Bytes())
//			binary.Write(buf11, endian, outmsg.QuatMotor[2]) //
//			copy(outbuf[56:], buf11.Bytes())
//			binary.Write(buf12, endian, outmsg.QuatMotor[3]) //
//			copy(outbuf[60:], buf12.Bytes())

//			binary.Write(buf13, endian, outmsg.EularMotor[0]) //
//			copy(outbuf[64:], buf13.Bytes())
//			binary.Write(buf14, endian, outmsg.EularMotor[1]) //
//			copy(outbuf[68:], buf14.Bytes())
//			binary.Write(buf15, endian, outmsg.EularMotor[2]) //
//			copy(outbuf[72:], buf15.Bytes())
//			binary.Write(buf16, endian, outmsg.EularMotor[3]) //
//			copy(outbuf[76:], buf16.Bytes())

//			// write a message to server
//			_, err = conn.Write([]byte(outbuf)) //_, err = listener.WriteToUDP([]byte("world"), remoteAddr)
//			if err != nil {
//				log.Println(err)
//			}
//			if state == true {
//				<-timer.C
//				if enable == false {

//					<-timer.C
//					break
//				}
//				break
//			}
//			<-timer.C
//			if enable == false {

//				<-timer.C
//				break
//			}
//		}
//		if enable == false {
//			break
//		}
//	}
//}
//func UdpEularMotorUseTheEular() {

//	remoteAddr, err := net.ResolveUDPAddr("udp", *raddr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}
//	tmpAddr := &net.UDPAddr{
//		IP:   net.ParseIP("127.0.0.1"),
//		Port: 0,
//	}
//	conn, err := net.DialUDP("udp", tmpAddr, remoteAddr)
//	if err != nil {
//		log.Fatalln("Error: ", err)
//	}
//	endian := binary.BigEndian
//	defer conn.Close()
//	for {
//		state = false
//		enable = true
//		for i := 0; i < 10; i++ {
//			timer := time.NewTimer(time.Millisecond * 20)
//			Rchai := -Nx[i] //°

//			//RH := math.Acos((72*72+72*72-(Nw[i]/5.17)*(Nw[i]/5.17))/(2*72*72)) / PI * 360 //
//			outmsg.EularMotor[1] = float32(Ny[i]) /* + 180    */ //+补偿
//			for outmsg.EularMotor[1] > 360 {
//				outmsg.EularMotor[1] = (outmsg.EularMotor[1] - 360)
//			}
//			for outmsg.EularMotor[1] < 0 {
//				outmsg.EularMotor[1] = outmsg.EularMotor[1] + 360
//			}

//			oular := float32(Rchai) //+补偿
//			for oular > 360 {
//				oular = oular - 360
//			}
//			for oular < 0 {
//				oular = oular + 360
//			}
//			outmsg.EularMotor[0] = oular
//			outmsg.EularMotor[2] = float32(Nz[i]) //+补偿
//			for outmsg.EularMotor[2] > 360 {
//				outmsg.EularMotor[2] = outmsg.EularMotor[2] - 360
//			}
//			for outmsg.EularMotor[2] < 0 {
//				outmsg.EularMotor[2] = outmsg.EularMotor[2] + 360
//			}
//			if outmsg.EularMotor[2] < 90 {
//				outmsg.EularMotor[2] = 360 - outmsg.EularMotor[2]
//			} else if outmsg.EularMotor[2] < 180 {
//				outmsg.EularMotor[2] = 360 - outmsg.EularMotor[2]
//			} else if outmsg.EularMotor[2] < 270 {
//				outmsg.EularMotor[2] = 360 - outmsg.EularMotor[2]
//			} else {
//				outmsg.EularMotor[2] = 360 - outmsg.EularMotor[2]
//			}
//			//			if oular < 90 {
//			//				outmsg.EularMotor[0] = 360 - oular
//			//			} else if oular < 180 {
//			//				outmsg.EularMotor[0] = 180 + oular
//			//			} else if oular < 270 {
//			//				outmsg.EularMotor[0] = oular - 180
//			//			} else {
//			//				outmsg.EularMotor[0] = 360 - oular
//			//			}

//			//log.Println(outmsg.EularMotor)
//			Quat := make([]float32, 4)
//			Quat[0] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[1] = float32(math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[2] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2]*PI/360.0)) + math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2])*PI/360.0))
//			Quat[3] = float32(math.Cos(float64(outmsg.EularMotor[0]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[1]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[2]*PI/360.0)) - math.Sin(float64(outmsg.EularMotor[0]*PI/360.0))*math.Sin(float64(outmsg.EularMotor[1]*PI/360.0))*math.Cos(float64(outmsg.EularMotor[2])*PI/360.0))
//			outmsg.QuatMotor[0] = Quat[3]
//			outmsg.QuatMotor[1] = Quat[0]
//			outmsg.QuatMotor[2] = Quat[1]
//			outmsg.QuatMotor[3] = Quat[2]
//			//og.Println(i, outmsg.EularMotor, outmsg.QuatMotor)
//			outmsg.TimeStamp = int64(int(time.Now().Weekday())*24*3600+
//				time.Now().Hour()*3600+time.Now().Minute()*60+
//				time.Now().Second()*1000) + int64(time.Now().Nanosecond()/1000000)
//			outbuf := make([]byte, binary.Size(outmsg), binary.Size(outmsg))
//			buf0 := new(bytes.Buffer)
//			buf1 := new(bytes.Buffer)
//			buf2 := new(bytes.Buffer)
//			buf3 := new(bytes.Buffer)
//			buf4 := new(bytes.Buffer)
//			buf5 := new(bytes.Buffer)
//			buf6 := new(bytes.Buffer)
//			buf7 := new(bytes.Buffer)
//			buf8 := new(bytes.Buffer)
//			buf9 := new(bytes.Buffer)
//			buf10 := new(bytes.Buffer)
//			buf11 := new(bytes.Buffer)
//			buf12 := new(bytes.Buffer)
//			buf13 := new(bytes.Buffer)
//			buf14 := new(bytes.Buffer)
//			buf15 := new(bytes.Buffer)
//			buf16 := new(bytes.Buffer)
//			//头部
//			outbuf[0] = byte('R')
//			outbuf[1] = byte('E')
//			outbuf[2] = byte('P')
//			outbuf[3] = byte('T')
//			//预留
//			outbuf[4] = 0
//			outbuf[5] = 3
//			//贞长
//			outmsg.Size = 80

//			binary.Write(buf0, endian, outmsg.Size) //
//			copy(outbuf[6:], buf0.Bytes())
//			//四元素
//			binary.Write(buf1, endian, outmsg.QuatSensor[0]) //
//			copy(outbuf[16:], buf1.Bytes())
//			binary.Write(buf2, endian, outmsg.QuatSensor[1]) //
//			copy(outbuf[20:], buf2.Bytes())
//			binary.Write(buf3, endian, outmsg.QuatSensor[2]) //
//			copy(outbuf[24:], buf3.Bytes())
//			binary.Write(buf4, endian, outmsg.QuatSensor[3]) //
//			copy(outbuf[28:], buf4.Bytes())

//			binary.Write(buf5, endian, outmsg.EularSensor[0]) //
//			copy(outbuf[32:], buf5.Bytes())
//			binary.Write(buf6, endian, outmsg.EularSensor[1]) //
//			copy(outbuf[36:], buf6.Bytes())
//			binary.Write(buf7, endian, outmsg.EularSensor[2]) //
//			copy(outbuf[40:], buf7.Bytes())
//			binary.Write(buf8, endian, outmsg.EularSensor[3]) //
//			copy(outbuf[44:], buf8.Bytes())

//			binary.Write(buf9, endian, outmsg.QuatMotor[0]) //
//			copy(outbuf[48:], buf9.Bytes())
//			binary.Write(buf10, endian, outmsg.QuatMotor[1]) //
//			copy(outbuf[52:], buf10.Bytes())
//			binary.Write(buf11, endian, outmsg.QuatMotor[2]) //
//			copy(outbuf[56:], buf11.Bytes())
//			binary.Write(buf12, endian, outmsg.QuatMotor[3]) //
//			copy(outbuf[60:], buf12.Bytes())

//			binary.Write(buf13, endian, outmsg.EularMotor[0]) //
//			copy(outbuf[64:], buf13.Bytes())
//			binary.Write(buf14, endian, outmsg.EularMotor[1]) //
//			copy(outbuf[68:], buf14.Bytes())
//			binary.Write(buf15, endian, outmsg.EularMotor[2]) //
//			copy(outbuf[72:], buf15.Bytes())
//			binary.Write(buf16, endian, outmsg.EularMotor[3]) //
//			copy(outbuf[76:], buf16.Bytes())

//			// write a message to server
//			_, err = conn.Write([]byte(outbuf)) //_, err = listener.WriteToUDP([]byte("world"), remoteAddr)
//			if err != nil {
//				log.Println(err)
//			}
//			if state == true {
//				<-timer.C
//				if enable == false {

//					<-timer.C
//					break
//				}
//				break
//			}
//			<-timer.C
//			if enable == false {

//				<-timer.C
//				break
//			}
//		}
//		if enable == false {
//			break
//		}
//	}
//}
//func TEST() {

//	go UdpEularMotorUseTheEular()
//	var R0, R1, R2, R3 float64
//	var dat bool
//	for {
//		log.Println("please input the Elar of Rz")
//		timer := time.NewTimer(time.Millisecond * 10)
//		fmt.Scanln(&R0)
//		fmt.Scanln(&R1)
//		//R1 = R1 + 0.1
//		//R2 = R2 + 0.1
//		fmt.Scanln(&R2)
//		R3 = 0
//		if dat == true {
//			R3 = R3 + 0.3
//			if R3 > 220 {

//				dat = false
//			}
//		}
//		if dat == false {
//			R3 = R3 - 0.3
//			if R3 < 0 {

//				dat = true
//			}
//		}

//		FeedBackfilter(R0, R1, R2, R3)
//		<-timer.C

//	}
//}
