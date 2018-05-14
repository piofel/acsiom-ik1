#include "Messages.h"

using namespace std;

Messages::Messages(void)
{
}

void Messages::display_message(std::string message)
{
	cout << "Acsiom system message: " << message << "." << endl;
}

void Messages::display_message(std::string message, std::string message_location)
{
	display_message(message);
	display_message_location(message_location);
}

void Messages::display_message_with_integer(std::string message, std::string message_location, Integer number)
{
	cout << "Acsiom system message: " << message << ": " << number << "." << endl;
	display_message_location(message_location);
}

void Messages::display_message_location(std::string message_location)
{
	cout << "\tMessage location: " << message_location << endl;
}
