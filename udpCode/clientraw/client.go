package main

import (
	"fmt"
	"log"
	"net"
	"time"
)

func main() {
	remoteAddr, err := net.ResolveUDPAddr("udp", "127.0.0.1:11011")
	if err != nil {
		fmt.Println("Error: ", err)
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
	for {
		time.Sleep(time.Second * 3)
		conn.Write([]byte("hello"))
	}
}
