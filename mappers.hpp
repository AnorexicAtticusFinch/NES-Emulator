#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <string>

class MAPPER
{
	public: // FUNCTIONS

	MAPPER(uint8_t num_banks_PRG, uint8_t num_banks_CHR)
	{
		this->num_banks_PRG = num_banks_PRG;
		this->num_banks_CHR = num_banks_CHR;

		reset();
	}

	virtual bool cpu_map_read(uint16_t, uint32_t&) = 0;
	virtual bool cpu_map_write(uint16_t, uint32_t&) = 0;

	virtual bool ppu_map_read(uint16_t, uint32_t&) = 0;
	virtual bool ppu_map_write(uint16_t, uint32_t&) = 0;

	void reset()
	{}

//-------------------------------------------------------

	protected: // VARIABLES

	uint8_t num_banks_PRG = 0;
	uint8_t num_banks_CHR = 0;
};

//==================================================================================================================================

class MAPPER_0 : public MAPPER
{
	public: // FUNCTIONS

	MAPPER_0(uint8_t num_banks_PRG, uint8_t num_banks_CHR) : MAPPER(num_banks_PRG, num_banks_CHR)
	{}

	bool cpu_map_read(uint16_t, uint32_t&); // Override
	bool cpu_map_write(uint16_t, uint32_t&); // Override

	bool ppu_map_read(uint16_t, uint32_t&); // Override
	bool ppu_map_write(uint16_t, uint32_t&); // Override
};

bool MAPPER_0::cpu_map_read(uint16_t address, uint32_t &mapped_address)
{
	if (address >= 0x8000 && address <= 0xFFFF)
	{
		if (num_banks_PRG > 1)
		{
			mapped_address = address & 0x7FFF;
		}
		else
		{
			mapped_address = address & 0x3FFF;
		}

		return true;
	}
	return false;
}

bool MAPPER_0::cpu_map_write(uint16_t address, uint32_t &mapped_address)
{
	if (address >= 0x8000 && address <= 0xFFFF)
	{
		if (num_banks_PRG > 1)
		{
			mapped_address = address & 0x7FFF;
		}
		else
		{
			mapped_address = address & 0x3FFF;
		}

		return true;
	}
	return false;
}

bool MAPPER_0::ppu_map_read(uint16_t address, uint32_t &mapped_address)
{
	if (address <= 0x1FFF)
	{
		mapped_address = address;
		return true;
	}
	return false;
}

bool MAPPER_0::ppu_map_write(uint16_t address, uint32_t &mapped_address)
{
	if (address <= 0x1FFF)
	{
		if (!num_banks_CHR)
		{
			mapped_address = address;
			return true;
		}
	}
	return false;
}

//==================================================================================================================================