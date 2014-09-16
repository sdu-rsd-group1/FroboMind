#include "error_code.hpp"

char* err2msg(unsigned int code)
{
	unsigned int new_code = 0;
	char* msg;
	if(code < 0x100)
	{
		msg = "HMI: ";
		new_code = code;
	}
	else if(code < 0x200)
	{
		msg = "Robotics: ";
		new_code = code-0x100;
	}
	else if(code < 0x300)
	{
		msg = "Vision: ";
		new_code = code-0x200;
	}

    for (int i = 0; err_code[i].name; ++i)
        if (err_code[i].value == new_code)
            return err_code[i].name;
    return "unknown";
}

//----------------------------------------------------------------------//

error_code err_code[] = {
    { 0x00, "Started up" },
    { 0x01, "Halted" }
};


