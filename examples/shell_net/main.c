#include <stdio.h>

#include "shell.h"

#include <stdlib.h>
#include "xtimer.h"

//#include "lwip.h"
#include "arch/sys_arch.h" //for sys_lock/unlock_tcpip_core

#include "lwip/netif.h"

//#include "net/ipv4.h"

#include "net/sock/udp.h"  //sock_udp_ep_t
//#include "net/sock/util.h"
//#include <arpa/inet.h> // inet_pton

// main.c


#define _TEST_ADDR4_REMOTE  (0x014fa8c0U)   /* 192.168.79.1*/

static int send_command(int argc, char **argv)
{
 (void)argc;
 (void)argv;

 static sock_udp_t _sock;
 int result;

  static const sock_udp_ep_t remote = { .addr = { .ipv4_u32 = _TEST_ADDR4_REMOTE
 },
                                          .family = AF_INET,
                                          //.netif = _TEST_NETIF,
                                          .port = 555}; //_TEST_PORT_REMOTE };

 result=sock_udp_create(&_sock, NULL, NULL, SOCK_FLAGS_REUSE_EP);
 printf("sock_udp_create result: %d\n", result);

 if(argc >= 2)
     result=sock_udp_send(&_sock, argv[1], strlen(argv[1]), &remote);
 else
     result=sock_udp_send(&_sock, "ABCD", sizeof("ABCD"), &remote);
 printf("sock_udp_send result: %d\n", result);

 return 0;
} 


static const shell_command_t shell_commands[] = {
    { "send", "send UDP packet", send_command},
    { NULL, NULL, NULL }
};

int main(void)
{

 puts("Try help and sm commands!");
 printf("THREAD_STACKSIZE_DEFAULT %d bytes \n", THREAD_STACKSIZE_DEFAULT);
 
#define _TEST_ADDR4_LOCAL  (0x964fa8c0U)   /* 192.168.79.150 */
#define _TEST_ADDR4_MASK   (0x00ffffffU)   /* 255.255.255.0 */
sys_lock_tcpip_core();
struct netif *iface = netif_find("ET0");

if(iface != NULL)
{
 ip4_addr_t ip, subnet;
 ip.addr = _TEST_ADDR4_LOCAL;
 subnet.addr = _TEST_ADDR4_MASK;
 netif_set_addr(iface, &ip, &subnet, NULL);
 sys_unlock_tcpip_core();
}

 char line_buf[SHELL_DEFAULT_BUFSIZE];
 shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

 /* should never be reached */
 return 0;
}
