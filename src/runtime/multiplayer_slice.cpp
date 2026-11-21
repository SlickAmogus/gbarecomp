#include "multiplayer_slice.h"
#include "arm_decode.h"
#include "thumb_decode.h"
#include "gba_bus.h"
#include <unordered_map>
#include <array>
#include <memory>

namespace gbarecomp {
namespace {
bool opcode(const gba::GbaBus& b,std::uint32_t pc,bool thumb,std::uint32_t& word) {
    const auto width=thumb ? 2u : 4u;
    const std::uint8_t* bytes=nullptr; std::uint32_t off=0;
    if (pc<0x4000 && pc+width<=0x4000 && b.bios() && b.bios()->loaded()) {
        word=thumb ? b.bios()->read16(pc) : b.bios()->read32(pc); return true;
    }
    if (pc>=0x02000000 && pc<0x03000000) { bytes=b.ewram_ptr(); off=pc&0x3ffff; }
    else if (pc>=0x03000000 && pc<0x04000000) { bytes=b.iwram_ptr(); off=pc&0x7fff; }
    else if (pc>=0x08000000 && pc<0x0e000000) {
        off=pc&0x01ffffff;
        if (!b.rom_ptr() || off+width>b.rom_size()) return false;
        bytes=b.rom_ptr();
    }
    if (!bytes || pc&(width-1)) return false;
    word=0; for (unsigned i=0;i<width;++i) word|=std::uint32_t(bytes[off+i])<<(i*8);
    return true;
}
bool plain_memory(std::uint32_t lo,std::uint32_t hi,bool load) {
    if (hi<lo) return false;
    if (lo>=0x02000000 && hi<0x04000000) return true;
    if (lo>=0x05000000 && hi<0x08000000) return true;
    // Conservative cartridge subset: no GPIO, EEPROM, save or unmapped bus.
    return load && lo>=0x08000100 && hi<0x0a000000;
}
}
bool multiplayer_slice_safe(const gba::GbaBus& bus,const ArmCpuState& cpu) {
    using namespace armv4t;
    const auto pc=cpu.R[15]; const bool thumb=cpu.cpsr&CPSR_T_BIT;
    std::uint32_t word;
    if (!opcode(bus,pc,thumb,word)) return false;
    // Thumb formats 1..5 contain only ALU/register/branch operations. These
    // dominate ordinary game code and need no decoded-memory metadata.
    if (thumb && word < 0x4800u) return true;
    // ARM fast path, straight from the opcode bits (this runs before every
    // guest instruction of a link session, and ARM inner loops such as IWRAM
    // rasterisers otherwise pay a hash lookup each time). Same answers as the
    // decoded path below; anything not listed here falls through to it.
    if (!thumb && (word>>28)!=0xF) {
        const auto cls=word&0x0E000000u;
        if (cls==0x0A000000u) return true;                       // B, BL
        if (cls==0x02000000u) return true;                       // data processing / MSR, immediate
        if (cls==0x00000000u) {
            // Register-operand data processing, multiplies, BX, MRS/MSR are
            // plain; halfword/signed transfers and SWP (bit 7 and bit 4 set,
            // with a non-zero SH field or the SWP pattern) are memory accesses.
            if ((word&0x90u)!=0x90u) return true;
            if ((word&0x60u)==0 && (word&0x0FB00FF0u)!=0x01000090u) return true;   // MUL/MLA/long multiplies
        } else if (cls==0x04000000u) {
            // LDR/STR/LDRB/STRB, immediate offset.
            const unsigned rn=(word>>16)&15;
            auto base=rn==15 ? pc+8u : cpu.R[rn];
            const auto off=word&0xFFFu;
            const auto address=(word&(1u<<24)) ? ((word&(1u<<23)) ? base+off : base-off) : base;
            return plain_memory(address&~3u,(address&~3u)+3,(word&(1u<<20))!=0);
        }
    }
    // Classification depends on opcode bits and live registers, not on the
    // decode PC. Thumb's finite opcode space permits direct lookup instead of
    // hashing the guest PC at every generated instruction. Read actual bytes
    // first so RAM code changes and different cartridges remain isolated.
    struct Cached { std::uint32_t word; Instr instruction; };
    static std::unordered_map<std::uint64_t,Cached> cache;
    static std::array<std::unique_ptr<Instr>,65536> thumb_cache;
    const Instr* instruction;
    if (thumb) {
        auto& slot=thumb_cache[word];
        if (!slot) slot=std::make_unique<Instr>(ThumbDecoder::decode(static_cast<std::uint16_t>(word),0));
        instruction=slot.get();
    } else {
        const auto key=std::uint64_t(word);
        auto found=cache.find(key);
        if (found==cache.end()) {
            if (cache.size()>16384) cache.clear();
            found=cache.insert_or_assign(key,Cached{word,ArmDecoder::decode(word,0)}).first;
        }
        instruction=&found->second.instruction;
    }
    const auto& i=*instruction;
    if (i.is_undefined) return false;
    const auto reg=[&](unsigned n) { return n==15 ? pc+(thumb ? 4u : 8u) : cpu.R[n]; };
    switch (i.op) {
    case IrOp::LDR: case IrOp::LDRB: case IrOp::LDRH: case IrOp::LDRSB: case IrOp::LDRSH:
    case IrOp::STR: case IrOp::STRB: case IrOp::STRH: {
        const bool load=i.op!=IrOp::STR && i.op!=IrOp::STRB && i.op!=IrOp::STRH;
        auto base=reg(i.mem.rn);
        if (thumb && i.mem.rn==15) base&=~3u;
        auto off=i.mem.imm_offset;
        if (i.mem.by_register) {
            const auto& shift=i.mem.reg_offset;
            if (shift.by_register || shift.type!=ShiftType::LSL || shift.imm_or_rs>4) return false;
            off=reg(shift.rm)<<shift.imm_or_rs;
        }
        const auto address=i.mem.pre_indexed ? (i.mem.add ? base+off : base-off) : base;
        // Cover byte, halfword, aligned-word and signed-unaligned accesses.
        return plain_memory(address&~3u,(address&~3u)+3,load);
    }
    case IrOp::LDM: case IrOp::STM: {
        const auto base=reg(i.block.rn);
        return base>=64 && plain_memory(base-64,base+64,i.op==IrOp::LDM);
    }
    case IrOp::AND: case IrOp::EOR: case IrOp::SUB: case IrOp::RSB:
    case IrOp::ADD: case IrOp::ADC: case IrOp::SBC: case IrOp::RSC:
    case IrOp::TST: case IrOp::TEQ: case IrOp::CMP: case IrOp::CMN:
    case IrOp::ORR: case IrOp::MOV: case IrOp::BIC: case IrOp::MVN:
    case IrOp::B: case IrOp::BL: case IrOp::BX: case IrOp::BL_prefix: case IrOp::BL_suffix:
    case IrOp::MUL: case IrOp::MLA: case IrOp::UMULL: case IrOp::UMLAL: case IrOp::SMULL: case IrOp::SMLAL:
    case IrOp::MRS: case IrOp::MSR:
        return true;
    default: return false;
    }
}
}
