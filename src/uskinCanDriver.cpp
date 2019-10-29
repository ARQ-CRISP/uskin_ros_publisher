#include <string>
#include <iostream>
#include "uskin_publisher/uskinCanDriver.h"

UskinCanDriver::UskinCanDriver(){};
UskinCanDriver::~UskinCanDriver(){};

void input_log(std::string message)
{
  if (DEBUG)
    std::cout << "  >>" << message << '\n';
}

void output_log(std::string message)
{
  if (DEBUG)
    std::cout << "  <<" << message << '\n';
}

int UskinCanDriver::open_connection()
{
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("Error in socket bind");
    return -2;
  }

  return 0;
}

void UskinCanDriver::send_message(can_frame sending_frame)
{

  int nbytes;

  nbytes = write(s, &sending_frame, sizeof(struct can_frame));

  printf("Sent %d bytes\n", nbytes);

  return;
}

int UskinCanDriver::read_message(can_frame *receiving_frame)
{

  int nbytes;
  struct sockaddr_can addr;

  socklen_t len = sizeof(addr);

  //nbytes = read(s, receiving_frame, sizeof(struct can_frame));
  nbytes = recvfrom(s, receiving_frame, sizeof(struct can_frame),
                    0, (struct sockaddr *)&addr, &len);

  if (nbytes < 0)
  {
    perror("can raw socket read");
    return -1;
  }

  /* paranoid check ... */
  if (nbytes < sizeof(struct can_frame))
  {
    fprintf(stderr, "read: incomplete CAN frame\n");
    return -2;
  }

  return 1;
}

void UskinCanDriver::request_data(__u32 can_id = 0x201)
{
  struct can_frame sending_frame;

  sending_frame.can_id = (canid_t)can_id;
  sending_frame.can_dlc = 2;
  sending_frame.data[0] = 0x07;
  sending_frame.data[1] = 0x00;

  send_message(sending_frame);

  UskinCanDriver::data_requested = true;

  return;
}

void UskinCanDriver::stop_data(__u32 can_id = 0x201)
{
  struct can_frame sending_frame;

  sending_frame.can_id = (canid_t)can_id;
  sending_frame.can_dlc = 2;
  sending_frame.data[0] = 0x07;
  sending_frame.data[1] = 0x01;

  send_message(sending_frame);

  UskinCanDriver::data_requested = false;

  return;
}

void UskinCanDriver::read_data(uskin_xyz_data *instant_reading)
{
  struct can_frame receiving_frame;

  input_log(" UskinCanDriver:read_data()");

  if (!UskinCanDriver::data_requested)
  {
    // TO DO
    perror("You must first request data from the sensor");
    return;
  }

  if (is_filter_set) // reset socket options if no filter is defined
  {
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    is_filter_set = false;
  }

  instant_reading->clear();

  if (!UskinCanDriver::read_message(&receiving_frame))
    fprintf(stderr, "Problems reading data\n");

  printf("can_id: %X: \n", receiving_frame.can_id);
  for (int i = 0; i < 8; i++)
    printf("%02X ", receiving_frame.data[i]);

  output_log(" UskinCanDriver:read_data()");

  return;
}

// Read data from a specified list of CAN ID's
/* void UskinCanDriver::read_data(uskin_xyz_data * uskin_xyz_data, struct can_filter * rfilter, long number_of_filters)
{
  struct can_frame receiving_frame;
  
  // set filter independently of state of is_filter_set
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, number_of_filters);
  is_filter_set = true;    

  
  input_log(" UskinCanDriver:read_data()");


  if (!UskinCanDriver::data_requested)
  {
    perror("You must first request data from the sensor");
    return;
  }

  if(!UskinCanDriver::read_message(&receiving_frame))
    fprintf(stderr, "Problems reading data\n");


  printf("can_id: %X: ", receiving_frame.can_id);
  for (int i = 0; i < 8; i++)
          printf("%02X ", receiving_frame.data[i]);
  printf("\n");

  printf("Converted value: %d\n", convert_16bit_hex_to_dec(&receiving_frame.data[5]));
  

  output_log(" UskinCanDriver:read_data()");

  return;

}
 */
