#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <string>

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include "mappers.hpp"

// FORWARD DECLARATION
class ROM;

union PPU_STATUS
{
	struct
	{
		uint8_t unused : 5;
		uint8_t sprite_overflow : 1;
		uint8_t sprite_zero_hit : 1;
		uint8_t vertical_blank : 1;
	};
	uint8_t reg = 0;
};

union PPU_MASK
{
	struct
	{
		uint8_t grayscale : 1;
		uint8_t render_background_left : 1;
		uint8_t render_sprites_left : 1;
		uint8_t render_background : 1;
		uint8_t render_sprites : 1;
		uint8_t enhance_red : 1;
		uint8_t enhance_green : 1;
		uint8_t enhance_blue : 1;
	};
	uint8_t reg = 0;
};

union PPU_CONTROL
{
	struct
	{
		uint8_t nametable_x : 1;
		uint8_t nametable_y : 1;
		uint8_t increment_mode : 1;
		uint8_t pattern_sprite : 1;
		uint8_t pattern_background : 1;
		uint8_t sprite_size : 1;
		uint8_t slave_mode : 1; // Unused
		uint8_t enable_nmi : 1;
	};
	uint8_t reg = 0;
};

union PPU_LOOPY
{
	struct
	{
		uint16_t coarse_x : 5;
		uint16_t coarse_y : 5;
		uint16_t nametable_x : 1;
		uint16_t nametable_y : 1;
		uint16_t fine_y : 3;
		uint16_t unused : 1;
	};
	uint16_t reg = 0;
};

struct PPU_OAM // 1 chunk of Object Attribute Memory (4 Bytes)
{
	uint8_t y;
	uint8_t id;
	uint8_t attribute;
	uint8_t x;
};

class PPU
{
	public: // VARIABLES

	bool nmi = false;
	bool frame_complete = false;

	uint8_t* oam = nullptr;
	uint8_t oam_address = 0;

	public: // FUNCTIONS
	
	PPU();

	void clock();
	void reset();

	void connect_ROM(ROM*);

	uint8_t cpu_read(uint16_t);
	void cpu_write(uint16_t, uint8_t);

	uint8_t ppu_read(uint16_t);
	void ppu_write(uint16_t, uint8_t);

	olc::Sprite& get_screen();
	olc::Pixel& get_colour_from_palette(uint8_t, uint8_t);

//-------------------------------------------------------

	private: // VARIABLES

	ROM* rom;

	int16_t scanline = 0;
	int16_t cycles = 0;

	uint8_t pattern_memory[2][4096];
	uint8_t nametable[2][1024];
	uint8_t palette[32];

	olc::Pixel palette_screen[0x40];
	olc::Sprite screen = olc::Sprite(256, 240);

	// Foreground Rendering
	PPU_OAM OAM[64];
	PPU_OAM sprite[8];
	uint8_t sprite_count = 0;
	uint8_t sprite_shifter_pattern_lo[8];
	uint8_t sprite_shifter_pattern_hi[8];

	bool sprite0_hit_possible = false;
	bool sprite0_render = false;

	PPU_STATUS status;
	PPU_MASK mask;
	PPU_CONTROL control;

	PPU_LOOPY vram_address;
	PPU_LOOPY tram_address; 
	uint8_t fine_x = 0;

	uint8_t address_latch = 0;
	uint8_t ppu_data_buffer = 0;

	// Background Rendering
	uint8_t bg_next_tile_id = 0;
	uint8_t bg_next_tile_attrib = 0;
	uint8_t bg_next_tile_lsb = 0;
	uint8_t bg_next_tile_msb = 0;
	uint16_t bg_shifter_pattern_lo = 0;
	uint16_t bg_shifter_pattern_hi = 0;
	uint16_t bg_shifter_attrib_lo = 0;
	uint16_t bg_shifter_attrib_hi = 0;
};

//==================================================================================================================================

struct ROM_HEADER
{
	char name[4];
	uint8_t prg_rom_chunks;
	uint8_t chr_rom_chunks;
	uint8_t mapper1;
	uint8_t mapper2;
	uint8_t prg_ram_size;
	uint8_t tv_system1;
	uint8_t tv_system2;
	char unused[5];
};

#define ROM_MIRROR uint8_t
#define HORIZONTAL 0
#define VERTICAL 1
#define ONESCREEN_LO 2
#define ONESCREEN_HI 3

class ROM
{
	public: // VARIABLES

	bool valid_rom = false;

	ROM_MIRROR mirror = HORIZONTAL;

	public: // FUNCTIONS

	ROM(std::string);
	~ROM()
	{
		delete mapper;
	}

	void reset()
	{
		if (mapper != nullptr)
		{
			mapper->reset();
		}
	}

	bool cpu_read(uint16_t, uint8_t&);
	bool cpu_write(uint16_t, uint8_t);

	bool ppu_read(uint16_t, uint8_t&);
	bool ppu_write(uint16_t, uint8_t);

//-------------------------------------------------------

	private: // VARIABLES

	MAPPER* mapper = nullptr;
	uint8_t mapper_id = 0;
	uint8_t num_banks_PRG = 0;
	uint8_t num_banks_CHR = 0;
	std::vector<uint8_t> data_PRG;
	std::vector<uint8_t> data_CHR;
};

//==================================================================================================================================
//==================================================================================================================================
// FUNCTION DEFINITIONS
//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================

PPU::PPU()
{
	memset((uint8_t*) OAM, 0, 64 * sizeof(PPU_OAM));
	oam = (uint8_t*) OAM;

	memset(sprite, 0xFF, 8 * sizeof(PPU_OAM));
	memset(sprite_shifter_pattern_lo, 0, 8);
	memset(sprite_shifter_pattern_hi, 0, 8);

	palette_screen[0x00] = olc::Pixel(84, 84, 84);
	palette_screen[0x01] = olc::Pixel(0, 30, 116);
	palette_screen[0x02] = olc::Pixel(8, 16, 144);
	palette_screen[0x03] = olc::Pixel(48, 0, 136);
	palette_screen[0x04] = olc::Pixel(68, 0, 100);
	palette_screen[0x05] = olc::Pixel(92, 0, 48);
	palette_screen[0x06] = olc::Pixel(84, 4, 0);
	palette_screen[0x07] = olc::Pixel(60, 24, 0);
	palette_screen[0x08] = olc::Pixel(32, 42, 0);
	palette_screen[0x09] = olc::Pixel(8, 58, 0);
	palette_screen[0x0A] = olc::Pixel(0, 64, 0);
	palette_screen[0x0B] = olc::Pixel(0, 60, 0);
	palette_screen[0x0C] = olc::Pixel(0, 50, 60);
	palette_screen[0x0D] = olc::Pixel(0, 0, 0);
	palette_screen[0x0E] = olc::Pixel(0, 0, 0);
	palette_screen[0x0F] = olc::Pixel(0, 0, 0);
	palette_screen[0x10] = olc::Pixel(152, 150, 152);
	palette_screen[0x11] = olc::Pixel(8, 76, 196);
	palette_screen[0x12] = olc::Pixel(48, 50, 236);
	palette_screen[0x13] = olc::Pixel(92, 30, 228);
	palette_screen[0x14] = olc::Pixel(136, 20, 176);
	palette_screen[0x15] = olc::Pixel(160, 20, 100);
	palette_screen[0x16] = olc::Pixel(152, 34, 32);
	palette_screen[0x17] = olc::Pixel(120, 60, 0);
	palette_screen[0x18] = olc::Pixel(84, 90, 0);
	palette_screen[0x19] = olc::Pixel(40, 114, 0);
	palette_screen[0x1A] = olc::Pixel(8, 124, 0);
	palette_screen[0x1B] = olc::Pixel(0, 118, 40);
	palette_screen[0x1C] = olc::Pixel(0, 102, 120);
	palette_screen[0x1D] = olc::Pixel(0, 0, 0);
	palette_screen[0x1E] = olc::Pixel(0, 0, 0);
	palette_screen[0x1F] = olc::Pixel(0, 0, 0);
	palette_screen[0x20] = olc::Pixel(236, 238, 236);
	palette_screen[0x21] = olc::Pixel(76, 154, 236);
	palette_screen[0x22] = olc::Pixel(120, 124, 236);
	palette_screen[0x23] = olc::Pixel(176, 98, 236);
	palette_screen[0x24] = olc::Pixel(228, 84, 236);
	palette_screen[0x25] = olc::Pixel(236, 88, 180);
	palette_screen[0x26] = olc::Pixel(236, 106, 100);
	palette_screen[0x27] = olc::Pixel(212, 136, 32);
	palette_screen[0x28] = olc::Pixel(160, 170, 0);
	palette_screen[0x29] = olc::Pixel(116, 196, 0);
	palette_screen[0x2A] = olc::Pixel(76, 208, 32);
	palette_screen[0x2B] = olc::Pixel(56, 204, 108);
	palette_screen[0x2C] = olc::Pixel(56, 180, 204);
	palette_screen[0x2D] = olc::Pixel(60, 60, 60);
	palette_screen[0x2E] = olc::Pixel(0, 0, 0);
	palette_screen[0x2F] = olc::Pixel(0, 0, 0);
	palette_screen[0x30] = olc::Pixel(236, 238, 236);
	palette_screen[0x31] = olc::Pixel(168, 204, 236);
	palette_screen[0x32] = olc::Pixel(188, 188, 236);
	palette_screen[0x33] = olc::Pixel(212, 178, 236);
	palette_screen[0x34] = olc::Pixel(236, 174, 236);
	palette_screen[0x35] = olc::Pixel(236, 174, 212);
	palette_screen[0x36] = olc::Pixel(236, 180, 176);
	palette_screen[0x37] = olc::Pixel(228, 196, 144);
	palette_screen[0x38] = olc::Pixel(204, 210, 120);
	palette_screen[0x39] = olc::Pixel(180, 222, 120);
	palette_screen[0x3A] = olc::Pixel(168, 226, 144);
	palette_screen[0x3B] = olc::Pixel(152, 226, 180);
	palette_screen[0x3C] = olc::Pixel(160, 214, 228);
	palette_screen[0x3D] = olc::Pixel(160, 162, 160);
	palette_screen[0x3E] = olc::Pixel(0, 0, 0);
	palette_screen[0x3F] = olc::Pixel(0, 0, 0);
}

void PPU::clock()
{
	auto IncrementScrollX = [&]()
	{
		if (mask.render_background || mask.render_sprites)
		{
			if (vram_address.coarse_x == 31)
			{
				vram_address.coarse_x = 0;
				vram_address.nametable_x = ~vram_address.nametable_x;
			}
			else
			{
				vram_address.coarse_x++;
			}
		}
	};

	auto IncrementScrollY = [&]()
	{
		if (mask.render_background || mask.render_sprites)
		{
			if (vram_address.fine_y < 7)
			{
				vram_address.fine_y++;
			}
			else
			{
				vram_address.fine_y = 0;
				if (vram_address.coarse_y == 29)
				{
					vram_address.coarse_y = 0;
					vram_address.nametable_y = ~vram_address.nametable_y;
				}
				else if (vram_address.coarse_y == 31)
				{
					vram_address.coarse_y = 0;
				}
				else
				{
					vram_address.coarse_y++;
				}
			}
		}
	};

	auto TransferaddressessX = [&]()
	{
		if (mask.render_background || mask.render_sprites)
		{
			vram_address.nametable_x = tram_address.nametable_x;
			vram_address.coarse_x = tram_address.coarse_x;
		}
	};

	auto TransferaddressessY = [&]()
	{
		if (mask.render_background || mask.render_sprites)
		{
			vram_address.fine_y = tram_address.fine_y;
			vram_address.nametable_y = tram_address.nametable_y;
			vram_address.coarse_y = tram_address.coarse_y;
		}
	};

	auto LoadBackgroundShifters = [&]()
	{	
		bg_shifter_pattern_lo = (bg_shifter_pattern_lo & 0xFF00) | bg_next_tile_lsb;
		bg_shifter_pattern_hi = (bg_shifter_pattern_hi & 0xFF00) | bg_next_tile_msb;

		bg_shifter_attrib_lo  = (bg_shifter_attrib_lo & 0xFF00) | ((bg_next_tile_attrib & 1) ? 0xFF : 0);
		bg_shifter_attrib_hi  = (bg_shifter_attrib_hi & 0xFF00) | ((bg_next_tile_attrib & 2) ? 0xFF : 0);
	};

	auto UpdateShifters = [&]()
	{
		if (mask.render_background)
		{
			bg_shifter_pattern_lo <<= 1;
			bg_shifter_pattern_hi <<= 1;

			bg_shifter_attrib_lo <<= 1;
			bg_shifter_attrib_hi <<= 1;
		}

		if (mask.render_sprites && cycles >= 1 && cycles < 258)
		{
			for (uint8_t a = 0; a < sprite_count; a++)
			{
				if (sprite[a].x > 0)
				{
					sprite[a].x--;
				}
				else
				{
					sprite_shifter_pattern_lo[a] <<= 1;
					sprite_shifter_pattern_hi[a] <<= 1;
				}
			}
		}
	};

	auto flipbyte = [](uint8_t b)
	{
		b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
		b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
		b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
		return b;
	};

	if (scanline >= -1 && scanline < 240)
	{		
		if (scanline == 0 && cycles == 0)
		{
			cycles = 1;
		}

		if (scanline == -1 && cycles == 1)
		{
			status.vertical_blank = 0;

			status.sprite_overflow = 0;
			memset(sprite_shifter_pattern_lo, 0, 8);
			memset(sprite_shifter_pattern_hi, 0, 8);

			status.sprite_zero_hit = 0;
		}

		if ((cycles >= 2 && cycles < 258) || (cycles >= 321 && cycles < 338))
		{
			UpdateShifters();

			switch ((cycles - 1) % 8)
			{
				case 0:
					LoadBackgroundShifters();
					bg_next_tile_id = ppu_read(0x2000 | (vram_address.reg & 0xFFF));
					break;

				case 2:
					bg_next_tile_attrib = ppu_read(0x23C0 | (vram_address.nametable_y << 11) | (vram_address.nametable_x << 10) | ((vram_address.coarse_y >> 2) << 3) | (vram_address.coarse_x >> 2));
				
					if (vram_address.coarse_y & 2)
					{
						bg_next_tile_attrib >>= 4;
					}
					if (vram_address.coarse_x & 2)
					{
						bg_next_tile_attrib >>= 2;
					}

					bg_next_tile_attrib &= 3;
					break;

				case 4: 
					bg_next_tile_lsb = ppu_read((control.pattern_background << 12) + ((uint16_t)bg_next_tile_id << 4) + (vram_address.fine_y));
					break;

				case 6:
					bg_next_tile_msb = ppu_read((control.pattern_background << 12) + ((uint16_t)bg_next_tile_id << 4) + (vram_address.fine_y) + 8);
					break;

				case 7:
					IncrementScrollX();
					break;
			}
		}

		if (cycles == 256)
		{
			IncrementScrollY();
		}

		if (cycles == 257)
		{
			LoadBackgroundShifters();
			TransferaddressessX();
		}

		if (cycles == 338 || cycles == 340)
		{
			bg_next_tile_id = ppu_read(0x2000 | (vram_address.reg & 0xFFF));
		}

		if (scanline == -1 && cycles >= 280 && cycles < 305)
		{
			TransferaddressessY();
		}

		// Foreground Rendering
		if (cycles == 257 && scanline >= 0)
		{
			memset(sprite, 0xFF, 8 * sizeof(PPU_OAM));
			sprite_count = 0;

			sprite0_hit_possible = false;

			uint8_t iter = 0;
			while (iter < 64 && sprite_count <= 8)
			{
				int16_t diff = ((int16_t) scanline - (int16_t) OAM[iter].y);
				if (diff >= 0 && diff < (control.sprite_size ? 16 : 8))
				{
					if (sprite_count < 8)
					{
						if (iter == 0)
						{
							sprite0_hit_possible = true;
						}

						memcpy(&sprite[sprite_count], &OAM[iter], sizeof(PPU_OAM));
					}
					sprite_count++;
				}
				iter++;
			}
			status.sprite_overflow = (sprite_count > 8);
			sprite_count = std::min(sprite_count, (uint8_t) 8);
		}

		if (cycles == 340)
		{
			for (uint8_t a = 0; a < sprite_count; a++)
			{
				uint8_t sprite_pattern_bits_lo, sprite_pattern_bits_hi;
				uint16_t sprite_pattern_addr_lo, sprite_pattern_addr_hi;

				if (!control.sprite_size)
				{
					// 8x8 mode
					if (!(sprite[a].attribute & 0x80))
					{
						// Sprite is normal
						sprite_pattern_addr_lo = (control.pattern_sprite << 12) | (sprite[a].id << 4) | (scanline - sprite[a].y);
					}
					else
					{
						// Sprite is flipped vertically
						sprite_pattern_addr_lo = (control.pattern_sprite << 12) | (sprite[a].id << 4) | (7 - scanline + sprite[a].y);
					}
				}
				else
				{
					// 8x16 mode
					if (!(sprite[a].attribute & 0x80))
					{
						// Sprite is normal
						if (scanline - sprite[a].y < 8)
						{
							// Top half
							sprite_pattern_addr_lo = ((sprite[a].id & 1) << 12) | ((sprite[a].id & 0xFE) << 4 ) | ((scanline - sprite[a].y) & 7 );
						}
						else
						{
							// Bottom half
							sprite_pattern_addr_lo = ((sprite[a].id & 1) << 12) | (((sprite[a].id & 0xFE) + 1) << 4 ) | ((scanline - sprite[a].y) & 7 );
						}
					}
					else
					{
						// Sprite is flipped vertically
						if (scanline - sprite[a].y < 8)
						{
							// Top half
							sprite_pattern_addr_lo = ((sprite[a].id & 1) << 12) | ((sprite[a].id & 0xFE) << 4 ) | ((7 - scanline + sprite[a].y) & 7 );
						}
						else
						{
							// Bottom half
							sprite_pattern_addr_lo = ((sprite[a].id & 1) << 12) | (((sprite[a].id & 0xFE) + 1) << 4 ) | ((7 - scanline + sprite[a].y) & 7 );
						}
					}
				}

				sprite_pattern_addr_hi = sprite_pattern_addr_lo + 8;

				sprite_pattern_bits_lo = ppu_read(sprite_pattern_addr_lo);
				sprite_pattern_bits_hi = ppu_read(sprite_pattern_addr_hi);

				if (sprite[a].attribute & 0x40)
				{
					// Sprite has to be flipped horizontally
					sprite_pattern_bits_lo = flipbyte(sprite_pattern_bits_lo);
					sprite_pattern_bits_hi = flipbyte(sprite_pattern_bits_hi);
				}

				sprite_shifter_pattern_lo[a] = sprite_pattern_bits_lo;
				sprite_shifter_pattern_hi[a] = sprite_pattern_bits_hi;
			}
		}
	}

	if (scanline >= 241 && scanline < 261)
	{
		if (scanline == 241 && cycles == 1)
		{
			status.vertical_blank = 1;
			if (control.enable_nmi) 
			{
				nmi = true;
			}
		}
	}

	uint8_t bg_pixel = 0;
	uint8_t bg_palette = 0;
	uint16_t bit_mux = 0x8000 >> fine_x;
	if (mask.render_background)
	{
		uint8_t p0_pixel = (bg_shifter_pattern_lo & bit_mux) > 0;
		uint8_t p1_pixel = (bg_shifter_pattern_hi & bit_mux) > 0;
		bg_pixel = (p1_pixel << 1) | p0_pixel;

		uint8_t bg_pal0 = (bg_shifter_attrib_lo & bit_mux) > 0;
		uint8_t bg_pal1 = (bg_shifter_attrib_hi & bit_mux) > 0;
		bg_palette = (bg_pal1 << 1) | bg_pal0;
	}

	uint8_t fg_pixel = 0;
	uint8_t fg_palette = 0;
	uint8_t fg_priority = 0;
	if (mask.render_sprites)
	{
		sprite0_render = false;
		for (uint8_t a = 0; a < sprite_count; a++)
		{
			if (!sprite[a].x)
			{
				uint8_t fg_pixel_lo = (sprite_shifter_pattern_lo[a] & 0x80) > 0;
				uint8_t fg_pixel_hi = (sprite_shifter_pattern_hi[a] & 0x80) > 0;
				fg_pixel = (fg_pixel_hi << 1) | fg_pixel_lo;

				fg_palette = (sprite[a].attribute & 3) + 4;
				fg_priority = !(sprite[a].attribute & 0x20);

				if (fg_pixel)
				{
					if (!a)
					{
						sprite0_render = true;
					}
					break;
				}
			}
		}
	}

	uint8_t pixel = 0;
	uint8_t palette = 0;
	if (!bg_pixel && fg_pixel)
	{
		pixel = fg_pixel;
		palette = fg_palette;
	}
	else if (bg_pixel && !fg_pixel)
	{
		pixel = bg_pixel;
		palette = bg_palette;
	}
	else if (bg_pixel && fg_pixel)
	{
		if (fg_priority)
		{
			pixel = fg_pixel;
			palette = fg_palette;
		}
		else
		{
			pixel = bg_pixel;
			palette = bg_palette;
		}

		if (sprite0_hit_possible && sprite0_render)
		{
			if (mask.render_background & mask.render_sprites)
			{
				if (~(mask.render_background_left | mask.render_sprites_left))
				{
					if (cycles >= 9 && cycles < 258)
					{
						status.sprite_zero_hit = 1;
					}
				}
				else
				{
					if (cycles >= 1 && cycles < 258)
					{
						status.sprite_zero_hit = 1;
					}
				}
			}
		}
	}

	screen.SetPixel(cycles - 1, scanline, get_colour_from_palette(palette, pixel));

	cycles++;
	if (cycles >= 341)
	{
		cycles = 0;
		scanline++;
		if (scanline >= 261)
		{
			scanline = -1;
			frame_complete = true;
		}
	}
}

void PPU::reset()
{
	scanline = 0;
	cycles = 0;

	status.reg = 0;
	mask.reg = 0;
	control.reg = 0;

	vram_address.reg = 0;
	tram_address.reg = 0;
	fine_x = 0;

	address_latch = 0;
	ppu_data_buffer = 0;

	bg_next_tile_id = 0;
	bg_next_tile_attrib = 0;
	bg_next_tile_lsb = 0;
	bg_next_tile_msb = 0;
	bg_shifter_pattern_lo = 0;
	bg_shifter_pattern_hi = 0;
	bg_shifter_attrib_lo = 0;
	bg_shifter_attrib_hi = 0;
}

void PPU::connect_ROM(ROM* rom)
{
	this->rom = rom;
}

uint8_t PPU::cpu_read(uint16_t address)
{
	uint8_t data = 0;

	switch (address)
	{
		case 0: // Control - Not readable
			break; 

		case 1: // Mask - Not Readable
			break;

		case 2: // Status
			data = (status.reg & 0xE0) | (ppu_data_buffer & 0x1F);
			status.vertical_blank = 0;
			address_latch = 0;
			break;

		case 3: // OAM Address
			break;

		case 4: // OAM Data
			data = oam[oam_address];
			break;

		case 5: // Scroll - Not Readable
			break;

		case 6: // PPU addressess - Not Readable
			break;

		case 7: // PPU Data

			data = ppu_data_buffer;
			ppu_data_buffer = ppu_read(vram_address.reg);
			
			if (vram_address.reg >= 0x3F00)
			{
				data = ppu_data_buffer;
			}

			if (control.increment_mode)
			{
				vram_address.reg += 32;
			}
			else
			{
				vram_address.reg++;
			}
			break;
	}

	return data;
}

void PPU::cpu_write(uint16_t address, uint8_t data)
{
	switch (address)
	{
		case 0: // Control
			control.reg = data;
			tram_address.nametable_x = control.nametable_x;
			tram_address.nametable_y = control.nametable_y;
			break;

		case 1: // Mask
			mask.reg = data;
			break;

		case 2: // Status - Not Writable
			break;

		case 3: // OAM Address
			oam_address = data;
			break;

		case 4: // OAM Data
			oam[oam_address] = data;
			break;

		case 5: // Scroll
			if (!address_latch)
			{
				fine_x = data & 0x07;
				tram_address.coarse_x = data >> 3;
				address_latch = 1;
			}
			else
			{
				tram_address.fine_y = data & 0x07;
				tram_address.coarse_y = data >> 3;
				address_latch = 0;
			}
			break;

		case 6: // PPU addressess
			if (!address_latch)
			{
				tram_address.reg = (uint16_t)((data & 0x3F) << 8) | (tram_address.reg & 0x00FF);
				address_latch = 1;
			}
			else
			{
				tram_address.reg = (tram_address.reg & 0xFF00) | data;
				vram_address = tram_address;
				address_latch = 0;
			}
			break;

		case 7: // PPU Data
			ppu_write(vram_address.reg, data);

			if (control.increment_mode)
			{
				vram_address.reg += 32;
			}
			else
			{
				vram_address.reg++;
			}
			break;
	}
}

uint8_t PPU::ppu_read(uint16_t address)
{
	uint8_t data = 0;
	address &= 0x3FFF;

	if (rom->ppu_read(address, data))
	{
	}
	else if (address <= 0x1FFF)
	{
		data = pattern_memory[(address & 0x1000) >> 12][address & 0xFFF];
	}
	else if (0x2000 <= address && address <= 0x3EFF)
	{
		address &= 0xFFF;
		if (rom->mirror == VERTICAL)
		{
			if (address <= 0x3FF)
			{
				data = nametable[0][address & 0x3FF];
			}
			else if (address >= 0x400 && address <= 0x7FF)
			{
				data = nametable[1][address & 0x3FF];
			}
			else if (address >= 0x800 && address <= 0xBFF)
			{
				data = nametable[0][address & 0x3FF];
			}
			else if (address >= 0xC00 && address <= 0xFFF)
			{
				data = nametable[1][address & 0x3FF];
			}
		}
		else if (rom->mirror == HORIZONTAL)
		{
			if (address <= 0x3FF)
			{
				data = nametable[0][address & 0x3FF];
			}
			else if (address >= 0x400 && address <= 0x7FF)
			{
				data = nametable[0][address & 0x3FF];
			}
			else if (address >= 0x800 && address <= 0xBFF)
			{
				data = nametable[1][address & 0x3FF];
			}
			else if (address >= 0xC00 && address <= 0xFFF)
			{
				data = nametable[1][address & 0x3FF];
			}
		}
	}
	else if (0x3F00 <= address && address <= 0x3FFF)
	{
		address &= 0x1F;

		if (address == 0x10)
		{
			address = 0;
		}
		else if (address == 0x14)
		{
			address = 4;
		}
		else if (address == 0x18)
		{
			address = 8;
		}
		else if (address == 0x1C)
		{
			address = 12;
		}
		data = palette[address] & (mask.grayscale ? 0x30 : 0x3F);
	}

	return data;
}

void PPU::ppu_write(uint16_t address, uint8_t data)
{
	address &= 0x3FFF;

	if (rom->ppu_write(address, data))
	{
	}
	else if (address <= 0x1FFF)
	{
		pattern_memory[(address & 0x1000) >> 12][address & 0xFFF] = data;
	}
	else if (0x2000 <= address && address <= 0x3EFF)
	{
		address &= 0xFFF;
		if (rom->mirror == VERTICAL)
		{
			if (address <= 0x3FF)
			{
				nametable[0][address & 0x3FF] = data;
			}
			else if (address >= 0x400 && address <= 0x7FF)
			{
				nametable[1][address & 0x3FF] = data;
			}
			else if (address >= 0x800 && address <= 0xBFF)
			{
				nametable[0][address & 0x3FF] = data;
			}
			else if (address >= 0xC00 && address <= 0xFFF)
			{
				nametable[1][address & 0x3FF] = data;
			}
		}
		else if (rom->mirror == HORIZONTAL)
		{
			if (address <= 0x3FF)
			{
				nametable[0][address & 0x3FF] = data;
			}
			else if (address >= 0x400 && address <= 0x7FF)
			{
				nametable[0][address & 0x3FF] = data;
			}
			else if (address >= 0x800 && address <= 0xBFF)
			{
				nametable[1][address & 0x3FF] = data;
			}
			else if (address >= 0xC00 && address <= 0xFFF)
			{
				nametable[1][address & 0x3FF] = data;
			}
		}
	}
	else if (0x3F00 <= address && address <= 0x3FFF)
	{
		address &= 0x1F;

		if (address == 0x10)
		{
			address = 0;
		}
		else if (address == 0x14)
		{
			address = 4;
		}
		else if (address == 0x18)
		{
			address = 8;
		}
		else if (address == 0x1C)
		{
			address = 12;
		}
		palette[address] = data;
	}
}

olc::Sprite& PPU::get_screen()
{
	return screen;
}

olc::Pixel& PPU::get_colour_from_palette(uint8_t palette, uint8_t pixel)
{
	return palette_screen[ppu_read(0x3F00 + (palette << 2) + pixel) & 0x3F];
}

//==================================================================================================================================

ROM::ROM(std::string filename)
{
	std::ifstream file(filename, std::ifstream::binary);
	if (!file.is_open())
	{
		return;
	}
	
	ROM_HEADER header;
	file.read((char*) &header, sizeof(ROM_HEADER));

	if (header.mapper1 & 4)
	{
		// Skip 512 Bytes if "Trainer" data is present
		file.seekg(512, std::ios_base::cur);
	}

	mapper_id = ((header.mapper2 >> 4) << 4) | (header.mapper1 >> 4);

	if (header.mapper1 & 1)
	{
		mirror = VERTICAL;
	}
	else
	{
		mirror = HORIZONTAL;
	}

	uint8_t nFileType = 1;

	if (nFileType == 0)
	{
	}

	if (nFileType == 1)
	{
		num_banks_PRG = header.prg_rom_chunks;
		data_PRG.resize(16384 * num_banks_PRG);
		file.read((char*) data_PRG.data(), data_PRG.size());

		num_banks_CHR = header.chr_rom_chunks;
		if (!num_banks_CHR)
		{
			// Treat as RAM
			data_CHR.resize(8192);
		}
		else
		{
			data_CHR.resize(8192 * num_banks_CHR);
		}
		file.read((char*) data_CHR.data(), data_CHR.size());
	}

	if (nFileType == 2)
	{
	}

	switch (mapper_id)
	{
		case 0:
			mapper = new MAPPER_0(num_banks_PRG, num_banks_CHR);
			break;
		default:
			std::cout << "ERROR! UNSUPPORTED MAPPER " << (u_int) mapper_id << "!\n";
			break;
	}

	valid_rom = true;
	file.close();
}

bool ROM::cpu_read(uint16_t addresses, uint8_t &data)
{
	uint32_t mapped_addresses = 0;
	if (mapper->cpu_map_read(addresses, mapped_addresses))
	{
		data = data_PRG[mapped_addresses];
		return true;
	}
	return false;
}

bool ROM::cpu_write(uint16_t addressess, uint8_t data)
{
	uint32_t mapped_addressess = 0;
	if (mapper->cpu_map_write(addressess, mapped_addressess))
	{
		data_PRG[mapped_addressess] = data;
		return true;
	}
	return false;
}

bool ROM::ppu_read(uint16_t addressess, uint8_t & data)
{
	uint32_t mapped_addressess = 0;
	if (mapper->ppu_map_read(addressess, mapped_addressess))
	{
		data = data_CHR[mapped_addressess];
		return true;
	}
	return false;
}

bool ROM::ppu_write(uint16_t addressess, uint8_t data)
{
	uint32_t mapped_addressess = 0;
	if (mapper->ppu_map_write(addressess, mapped_addressess))
	{
		data_CHR[mapped_addressess] = data;
		return true;
	}
	return false;
}