#ifndef USKINCANDRIVER_H
#define USKINCANDRIVER_H

#include <ctime>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// CAN dependent libraries
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define DEBUG 0

void input_log(std::string message);

void output_log(std::string message);

struct uskin_single_node_reading
{
  __u32 id;
  unsigned int x_data;
  unsigned int y_data;
  unsigned int z_data;
};

typedef struct uskin_matrix_reading
{
  std::time_t timestamp;
  std::vector struct uskin_single_node_reading data[6][4];
} uskin_xyz_data;

class UskinCanDriver
{
  // Access specifier
private:
  int s;
  struct sockaddr_can addr;
  struct ifreq ifr;

  const char *ifname = "can0";

  bool data_requested = false;

  bool is_filter_set = false; // check if there is any filter applied to the socket (incoming data)

  void send_message(can_frame sending_frame);
  int read_message(can_frame receiving_frame);

public:
  UskinCanDriver();
  ~UskinCanDriver();

  int open_connection();

  void request_data(__u32 can_id);

  void stop_data(__u32 can_id);

  void read_data(uskin_xyz_data * instant_reading);
  
  void read_data(uskin_xyz_data * uskin_xyz_data, struct can_filter * rfilter, long number_of_filters);
};

#endif
