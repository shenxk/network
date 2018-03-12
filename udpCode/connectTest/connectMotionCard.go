package main

import "net"

import "errors"
import "log"

var m_ConnID int16
var m_CurrentConnType int

func main() {
	connID, e := OpenNet("192.168.16.8", "7408", "192.168.16.216", "8080")
	log.Println("good %s;%s", connID, e)
}

func OpenNet(raddr, rport, laddr, lport string) (connID int16, e error) {

	if m_ConnID != -1 {
		/*	return -1,*/ errors.New("Connection already exist!")
	}

	//addr_remote, e := net.ResolveUDPAddr("udp", "255.255.255.255"+":"+"7408")
	addr_remote, e := net.ResolveUDPAddr("udp", raddr+":"+rport)
	log.Printf("remot Udp addr %#v\n", addr_remote)
	if e != nil {
		return -1, e
	}

	if laddr != "" && lport != "" {
		//addr_local, e := net.ResolveUDPAddr("udp", "127.0.0.1"+":"+"8081")
		addr_local, e := net.ResolveUDPAddr("udp", laddr+":"+lport)
		if e != nil {
			return -1, e
		} else {
			mp_ConnUDP, e := net.DialUDP("udp", addr_local, addr_remote)
			log.Printf("remot Udp addr %#v\n", mp_ConnUDP)
			//println("UDP dial: ", raddr, rport, laddr, lport)
			if e != nil {
				return -1, e
			}
		}
	} else {
		mp_ConnUDP, e := net.DialUDP("udp", nil, addr_remote)
		log.Printf("remot Udp addr %#v\n", mp_ConnUDP)
		if e != nil {
			return -1, e
		}
	}

	//e = mp_ConnUDP.SetReadDeadline(time.Now().Add(5000 * time.Millisecond))
	//if e != nil {
	//	return -1, e
	//}

	m_ConnID = 0
	m_CurrentConnType = 2

	return m_ConnID, nil
}
