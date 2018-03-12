package main

import (
	"fmt"
	//	"io/ioutil"
	"net"
	"os"
	"time"
)

func main() {
	//	if len(os.Args) != 2 {
	//		fmt.Fprintf(os.Stderr, "Usage: %s host:port ", os.Args[0])
	//		os.Exit(1)
	//	}
	service := "192.168.3.114:7777"
	//	fmt.Println(service)
	tcpAddr, err := net.ResolveTCPAddr("tcp4", service)
	checkError(err)
	fmt.Printf("%#v", tcpAddr)
	conn, err := net.DialTCP("tcp", nil, tcpAddr)
	fmt.Printf("%#v", conn)
	checkError(err)
	for {

		_, err = conn.Write([]byte("hello"))
		checkError(err)
		time.Sleep(time.Second * 1)
		fmt.Println(string("hello"))
	}

	//os.Exit(0)
}
func checkError(err error) {
	if err != nil {
		fmt.Fprintf(os.Stderr, "Fatal error: %s", err.Error())
		os.Exit(1)
	}
}
