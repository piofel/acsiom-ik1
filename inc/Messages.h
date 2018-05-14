#ifndef __MESSAGES_H_INCLUDED__
#define __MESSAGES_H_INCLUDED__

#include <iostream>
#include "TypeDefinitions.h"

class Messages
{
	public:
		Messages(void);
		void display_message(std::string message);
		void display_message(std::string message, std::string message_location);
		void display_message_with_integer(std::string message, std::string message_location, Integer number);
		void display_message_location(std::string message_location);
};

#endif
