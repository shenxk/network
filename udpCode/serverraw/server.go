package main

import (
	"fmt"
	"log"
	"net"
	"time"
)

func main() {
	listener, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.ParseIP("127.0.0.1"), Port: 11011})
	if err != nil {
		fmt.Println(err)
		return
	}

	fmt.Printf("Local: <%s> \n", listener.LocalAddr().String())

	bufin := make([]byte, 1024)
	for {
		n, remoteAddr, err := listener.ReadFromUDP(bufin)
		if err != nil {
			log.Printf("error during read: %s", err)
		}
		log.Println(remoteAddr, string(bufin[:n]))
		time.Sleep(time.Millisecond * 1)
	}
}
