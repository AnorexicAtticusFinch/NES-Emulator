#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <string>

#include "bus_cpu.hpp"

class TEST : public olc::PixelGameEngine
{
	public: // FUNCTIONS

	TEST()
	{
		sAppName = "PPU Background Render Test";
	}
	~TEST()
	{
		delete rom;
	}

//-------------------------------------------------------

	private: // VARIABLES

	BUS nes;
	ROM* rom;

	float fResidualTime = 0;

	private: // FUNCTIONS

	bool OnUserCreate(); // Override
	bool OnUserUpdate(float); //Override

};

bool TEST::OnUserCreate()
{
	std::string filename;
	std::cout << "Enter Rom Path: ";
	std::cin >> filename;
	std::cout << "\n";
	rom = new ROM("Roms/" + filename + ".nes");
	if (!rom->valid_rom)
	{
		std::cout << "ERROR! INVALID ROM PATH!\n";
		return false;
	}

	std::cout << "A Button is: E\n";
	std::cout << "B Button is: Q\n";
	std::cout << "Select Button is: SHIFT\n";
	std::cout << "Start Button is: ENTER\n\n";

	nes.connect_ROM(rom);
	nes.reset();
	return true;
}

bool TEST::OnUserUpdate(float fElapsedTime)
{
	Clear(olc::BLACK);

	nes.controller[0] = 0x00;
	nes.controller[0] |= GetKey(olc::Key::E).bHeld ? 0x80 : 0x00; // A
	nes.controller[0] |= GetKey(olc::Key::Q).bHeld ? 0x40 : 0x00; // B
	nes.controller[0] |= GetKey(olc::Key::SHIFT).bHeld ? 0x20 : 0x00; // Select
	nes.controller[0] |= GetKey(olc::Key::ENTER).bHeld ? 0x10 : 0x00; // Start
	nes.controller[0] |= GetKey(olc::Key::UP).bHeld ? 0x08 : 0x00;
	nes.controller[0] |= GetKey(olc::Key::DOWN).bHeld ? 0x04 : 0x00;
	nes.controller[0] |= GetKey(olc::Key::LEFT).bHeld ? 0x02 : 0x00;
	nes.controller[0] |= GetKey(olc::Key::RIGHT).bHeld ? 0x01 : 0x00;

	if (fResidualTime > 0)
	{
		fResidualTime -= fElapsedTime;
	}
	else
	{
		fResidualTime += (1.0 / 60.0) - fElapsedTime;
		do
		{
			nes.clock();
		} while (!nes.ppu.frame_complete);
		nes.ppu.frame_complete = false;
	}

	DrawSprite(0, 0, &nes.ppu.get_screen(), 2);
	return true;
}

int main()
{
	TEST test;
	test.Construct(512, 480, 2, 2);
	test.Start();
}