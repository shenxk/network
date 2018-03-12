package main

import (
	"fmt"
	"log"
	"net"
	"os"
)

func main() {

	tcpAddr, err := net.ResolveTCPAddr("tcp4", "127.0.0.1:18889")
	checkError(err)
	listener, err := net.ListenTCP("tcp", tcpAddr)
	checkError(err)
	buf := make([]byte, 1024)

	for {
		conn, err := listener.Accept()
		log.Println("tcp 已连接")
		if err != nil {
			continue
		}
		//
		for {
			//daytime := time.Now().String()
			n, e := conn.Read(buf)
			if e != nil {
				log.Println("tcp 断开")
				conn.Close()
				break
			}
			fmt.Printf("__%s\n", string(buf[:n]))
			//			n, e = conn.Write([]byte("hello client:" + daytime)) // don't care about return value
			//			if e != nil {
			//				conn.Close()
			//				break
			//			}
		}

	}
}
func checkError(err error) {
	if err != nil {
		fmt.Fprintf(os.Stderr, "Fatal error: %s", err.Error())
		os.Exit(1)
	}
}
