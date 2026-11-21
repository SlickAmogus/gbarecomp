#include "multiplayer_launch.h"
#include "multiplayer_match.h"
#include "multiplayer_pacing.h"
#include "host_window.h"
#include "gba_bios.h"
#include "sha1.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

namespace gbarecomp {
namespace {
void configure_save(gba::GbaSave& save,const SaveConfiguration& config) {
    switch (config.type) {
    case gba::SaveType::SRAM: save.configure_sram(config.size); break;
    case gba::SaveType::EEPROM: save.configure_eeprom(config.size); break;
    case gba::SaveType::Flash512:
    case gba::SaveType::Flash1M: save.configure_flash(config.size); break;
    default: throw std::invalid_argument("netplay cartridge save type is not configured");
    }
}
void load_owner_save(gba::GbaSave& save,const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) return; // new cartridge
    std::ifstream input(path,std::ios::binary|std::ios::ate);
    if (!input) throw std::runtime_error("cannot open player's save: "+path.string());
    const auto size=input.tellg();
    if (size<=0 || size>128*1024) throw std::runtime_error("invalid player save length");
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))
        throw std::runtime_error("cannot read player's save");
    const bool loaded=save.flash_enabled() ? save.load_flash_bytes(bytes.data(),bytes.size()) :
        save.sram_enabled() ? save.load_sram_bytes(bytes.data(),bytes.size()) :
        save.load_eeprom_bytes(bytes.data(),bytes.size());
    if (!loaded) throw std::runtime_error("player save does not fit the cartridge");
    save.clear_dirty();
}
int run(const GbaNetplayLaunch& launch,const GbaNetplayBoot& boot) {
    validate_gba_netplay_launch(launch);
    const auto checkpoint_path=launch.checkpoint_path.empty() ?
        boot.config_directory/"netplay"/("session-"+std::to_string(launch.session_id)+".paired") : launch.checkpoint_path;
    const auto checkpoint_absolute=std::filesystem::weakly_canonical(checkpoint_path);
    for (const auto& original:{boot.local_save,boot.rom_path,boot.bios_path})
        if (!original.empty() && checkpoint_absolute==std::filesystem::weakly_canonical(original))
            throw std::invalid_argument("paired checkpoint must not replace a cartridge save, ROM or BIOS");
    std::filesystem::create_directories(checkpoint_absolute.parent_path());
    if (!boot.bios || !boot.rom || boot.bios->sha1_hex()!=gba::GbaBios::kExpectedSha1 ||
        gba::sha1(boot.rom->data(),boot.rom->size()).hex()!=boot.expected_rom_sha1)
        throw std::invalid_argument("netplay asset verification failed");
    if (!valid_save_size(boot.save.type,boot.save.size))
        throw std::invalid_argument("invalid netplay save hardware");
    const unsigned players=launch.player_count;
    GbaSessionConfig config;
    GbaLinkGroup link{launch.medium,{}};
    for (unsigned id=0;id<players;++id) {
        config.machines.push_back({id,launch.program_id,boot.expected_rom_sha1});
        link.machines.push_back(id); // machine ID == cable port
        config.input_machines.push_back(launch.seat_machine[id]);
    }
    config.links={std::move(link)};
    GbaMultiplayerSession simulation(std::move(config));
    simulation.set_native_slices(true);
    simulation.set_link_lookahead(launch.link_lookahead);
    for (unsigned id=0;id<players;++id) {
        auto& m=simulation.machine(id);
        m.ppu.set_rasterization_enabled(false);
        m.bus.set_bios(boot.bios); m.bus.set_rom(boot.rom->data(),boot.rom->size());
        configure_save(m.bus.save(),boot.save);
        if (launch.setup_instance) launch.setup_instance(m);
    }
    // Agreed opening: GBARECOMP_NETPLAY_OPENING names a text file of
    // "seat first_tick last_tick mask" lines, every seat's pad for the
    // match's first ticks (a game uses it to step all consoles through its
    // menus to the chosen mode). Every peer knows all of it, so each runs it
    // alone at full speed before connecting, from blank cartridges (owner
    // saves differ between players; blank ones make every peer compute the
    // same consoles). The match then starts there as a restored pair, whose
    // startup agreement checks that the peers really arrived at one state.
    struct OpeningRow { unsigned seat; std::uint32_t first, last; std::uint16_t mask; };
    std::vector<OpeningRow> opening;
    std::uint32_t opening_ticks=0;
    if (const char* path=std::getenv("GBARECOMP_NETPLAY_OPENING"); path && *path && launch.resume_path.empty()) {
        std::ifstream in(path);
        unsigned long seat=0,first=0,last=0; std::string mask;
        while (in>>seat>>first>>last>>mask) {
            if (seat>=players || last<first || last>=1000000)
                throw std::invalid_argument("invalid netplay opening row");
            opening.push_back({static_cast<unsigned>(seat),static_cast<std::uint32_t>(first),
                               static_cast<std::uint32_t>(last),
                               static_cast<std::uint16_t>(std::stoul(mask,nullptr,0)&0x3ff)});
            opening_ticks=std::max(opening_ticks,static_cast<std::uint32_t>(last+1));
        }
    }
    if (launch.resume_path.empty() && opening.empty())
        load_owner_save(simulation.input_machine(launch.local_seat).bus.save(),boot.local_save);
    // Reject unsupported peripheral/enhancement state before opening transport.
    simulation.save_state();
    GbaNetplayMatchOptions options;
    options.network.local_slot=launch.local_seat;
    options.network.session_id=launch.session_id;
    options.network.input_delay=launch.input_delay;
    options.network.slot_count=static_cast<rnet_u8>(players);
    options.network.occupied_mask=(1u<<players)-1;
    options.prediction=launch.prediction; options.rollback=launch.rollback;
    options.force_turn=launch.force_turn;
    options.planned_finish_tick=boot.finish_tick;
    options.identity=std::string(launch.medium==GbaLinkMedium::Wireless ? "gba-wireless/1:" : "gba-cable/1:")+launch.program_id+":"+launch.build_identity+":"+
        boot.expected_rom_sha1+":"+boot.bios->sha1_hex()+":"+
        std::to_string(static_cast<unsigned>(boot.save.type))+":"+std::to_string(boot.save.size)+":"+
        gba_netplay_seat_identity(launch)+":native-lle"+
        (launch.link_lookahead ? ":link-lookahead="+std::to_string(launch.link_lookahead) : std::string());
    const auto identity_digest=gba::sha1(reinterpret_cast<const std::uint8_t*>(options.identity.data()),options.identity.size());
    options.build_fingerprint=std::uint32_t(identity_digest.bytes[0])|
        (std::uint32_t(identity_digest.bytes[1])<<8)|(std::uint32_t(identity_digest.bytes[2])<<16)|
        (std::uint32_t(identity_digest.bytes[3])<<24);
    if (!options.build_fingerprint) options.build_fingerprint=1;
    // Wall time is sampled only here; startup exchanges each owner's seed.
    options.rtc_seed_seconds=std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (!launch.resume_path.empty()) {
        GbaConfirmedCheckpoint saved;
        std::string error;
        if (!gba_load_checkpoint_archive(launch.resume_path,options.identity,saved,&error) ||
            !simulation.load_state(saved.state,&error)) throw std::runtime_error("cannot resume paired session: "+error);
        options.restored_pair=true;
    }
    std::unique_ptr<GbaNetplayPresentation> presentation;
    const bool adaptive = launch.view == GbaNetplayView::Adaptive;
    const unsigned initial_width = gba_netplay_view_width(launch.view, 0, 0,
                                                          launch.view_policy.max_width);
    presentation = std::make_unique<GbaNetplayPresentation>(launch.view_policy, boot.affine_filter, boot.internal_resolution);
    presentation->request_width(initial_width);
    HostWindow window;
    if (!window.open(boot.scale,initial_width,160,boot.title.c_str(),boot.screen.c_str(),
            boot.linear_filter,boot.sharp_filter,adaptive,true,boot.audio_rate))
        throw std::runtime_error("cannot open netplay window");
    window.load_input_config(boot.config_directory.string().c_str(),false);
    window.set_fullscreen(boot.fullscreen); window.set_volume(boot.volume);
    window.set_fps_readout(boot.show_fps);
    if (!opening.empty()) {
        // No picture is drawn meanwhile (no presentation observer yet): the
        // game's present hook shows progress over a black screen.
        using Clock=std::chrono::steady_clock;
        const std::vector<std::uint8_t> black(240u*160u*3u,0);
        std::vector<std::uint16_t> pads(players,0);
        auto next_show=Clock::time_point{};
        auto leave_armed_until=Clock::time_point{};
        const auto started=Clock::now();
        for (std::uint32_t tick=0;tick<opening_ticks;++tick) {
            std::fill(pads.begin(),pads.end(),std::uint16_t{0});
            for (const OpeningRow& row:opening)
                if (tick>=row.first && tick<=row.last) pads[row.seat]=row.mask;
            simulation.run_frame(pads);
            simulation.discard_audio_output();
            const auto now=Clock::now();
            if (now<next_show && tick+1<opening_ticks) continue;
            next_show=now+std::chrono::milliseconds(33);
            auto events=window.pump();
            if (events.quit && events.escape && !launch.escape_leaves && !events.terminating) {
                if (now>=leave_armed_until) { leave_armed_until=now+std::chrono::seconds(3); events.quit=false; }
            }
            if (events.quit) { if (launch.on_end) launch.on_end(true,""); return 0; }
            if (events.toggle_fullscreen) window.set_fullscreen(window.fullscreen() ? 0 : 1);
            if (!window.set_surface_size(240,160)) throw std::runtime_error("cannot size netplay presentation surface");
            window.set_view_margins(0,0,0,0);
            if (launch.present_frame) {
                GbaNetplayPresentFrame frame;
                frame.guest_rgb=black.data(); frame.guest_width=240; frame.guest_height=160;
                frame.local_bus=&simulation.input_machine(launch.local_seat).bus;
                frame.tick=tick+1; frame.opening_end=opening_ticks;
                frame.leave_armed=now<leave_armed_until;
                if (!window.drawable_size(&frame.drawable_width,&frame.drawable_height) ||
                    frame.drawable_width<=0 || frame.drawable_height<=0) {
                    frame.drawable_width=960; frame.drawable_height=540;
                }
                if (launch.present_frame(frame) && frame.argb)
                    window.set_external_frame(frame.argb,frame.width,frame.height);
            }
            window.present(black.data(),240,160);
        }
        std::printf("netplay opening: %u ticks in %.1f s\n",opening_ticks,
            std::chrono::duration<double>(Clock::now()-started).count());
        std::fflush(stdout);
        options.restored_pair=true;
    }
    simulation.input_machine(launch.local_seat).ppu.set_presentation_observer(presentation.get());
    struct DetachPresentation {
        GbaMultiplayerSession& simulation;
        unsigned seat;
        ~DetachPresentation() { simulation.input_machine(seat).ppu.set_presentation_observer(nullptr); }
    } detach{simulation, launch.local_seat};
    GbaNetplayMatch match(simulation,std::move(options));
    const auto transport=gba_netplay_transport(launch);
    if ((transport==GbaNetplayTransport::Hub ?
            rnet_session_start_lan_hub(match.transport(),launch.bind_endpoint.c_str()) :
            rnet_session_start_lan(match.transport(),launch.bind_endpoint.c_str(),launch.peer_endpoint.c_str())))
        throw std::runtime_error("cannot open netplay transport");
    std::uint16_t buttons=0;
    // Test aid (scripted two-process runs with the SDL dummy driver):
    // GBARECOMP_NETPLAY_SCRIPT names a text file of "first_tick last_tick
    // mask" lines (mask = pad buttons held, KEYINPUT bit order, 1 = down);
    // inside a listed range the script replaces this seat's keyboard input.
    // Not only a test aid: games use it for an agreed opening sequence.
    struct ScriptRow { std::uint32_t first, last; std::uint16_t mask; };
    std::vector<ScriptRow> script;
    if (const char* path=std::getenv("GBARECOMP_NETPLAY_SCRIPT")) {
        std::ifstream in(path);
        unsigned long first=0,last=0; std::string mask;
        while (in>>first>>last>>mask)
            script.push_back({static_cast<std::uint32_t>(first),static_cast<std::uint32_t>(last),
                              static_cast<std::uint16_t>(std::stoul(mask,nullptr,0)&0x3ff)});
    }
    // Until the script's last tick the keyboard is ignored (no buttons between
    // rows); after it the player has the controls. A game uses this to step
    // both consoles through its menus to an agreed mode at the start.
    std::uint32_t script_end=0;
    for (const ScriptRow& row:script) script_end=std::max(script_end,row.last);
    match.sample_local=[&](std::uint32_t tick) {
        for (const ScriptRow& row:script)
            if (tick>=row.first && tick<=row.last) return row.mask;
        return (script.empty() || tick>script_end) ? buttons : std::uint16_t{0};
    };
    // Test aid: GBARECOMP_NETPLAY_DUMP=prefix writes the local picture as
    // prefix_<frame>.ppm every 60 presented frames, and each machine's EWRAM.
    const char* dump_prefix=std::getenv("GBARECOMP_NETPLAY_DUMP");
    unsigned long presented_frames=0;
    GbaNetplayPacer pacer;
    GbaNetplayOutput output;
    std::string previous_title;
    bool closing=false;
    bool ended=false;
    auto end=[&](bool local,const std::string& reason) {
        if (!ended && launch.on_end) launch.on_end(local,reason);
        ended=true;
    };
    // Esc with escape_leaves off: armed until this time, a second Esc leaves.
    auto leave_armed_until=GbaNetplayPacer::Clock::time_point{};
    auto next_wait_present=GbaNetplayPacer::Clock::time_point{};
    try {
    while (true) {
        if (launch.pump_lobby) launch.pump_lobby();
        auto events=window.pump();
        if (events.terminating) { end(true,""); return 0; } // keep the previous durable pair
        if (events.quit && events.escape && !launch.escape_leaves && !closing) {
            const auto t=GbaNetplayPacer::Clock::now();
            if (t>=leave_armed_until) { leave_armed_until=t+std::chrono::seconds(3); events.quit=false; }
        }
        if (events.quit) {
            // No save can be agreed with a peer that is not there.
            if (closing || match.phase()==GbaNetplayMatch::Phase::Starting ||
                match.connection().phase==GbaConnectionPhase::Reconnecting) { end(true,""); return 0; }
            closing=true; match.request_checkpoint();
        }
        if (events.save_slot && match.phase()==GbaNetplayMatch::Phase::Running) {
            closing=true; match.request_checkpoint();
        }
        buttons=static_cast<std::uint16_t>(~events.keyinput&0x3ff);
        if (events.toggle_fullscreen) window.set_fullscreen(window.fullscreen() ? 0 : 1);
        if (events.window_bigger) window.adjust_scale(1);
        if (events.window_smaller) window.adjust_scale(-1);
        if (events.volume_up) window.set_volume(window.volume()+5);
        if (events.volume_down) window.set_volume(window.volume()-5);
        if (events.toggle_fps) window.set_fps_readout(!window.fps_readout());
        if (adaptive && presentation) {
            int width=0, height=0;
            if (window.drawable_size(&width,&height))
                presentation->request_width(gba_netplay_view_width(launch.view,width,height,
                    launch.view_policy.max_width));
        }
        // Pause, turbo, rewind and single-machine load never reach this
        // simulation. A save hotkey requests a confirmed paired save-and-leave.
        const auto now=GbaNetplayPacer::Clock::now();
        const auto step=match.poll(pacer.ready(now));
        if (step==GbaNetplayMatch::Step::Forward) pacer.forwarded(now);
        if (match.phase()==GbaNetplayMatch::Phase::Failed) {
            end(false,match.error());
            throw std::runtime_error(match.error());
        }
        if (match.checkpoint_pending() && !boot.finish_tick) closing=true;
        if (match.phase()==GbaNetplayMatch::Phase::CheckpointReady) {
            std::string error;
            if (!match.store_checkpoint(checkpoint_absolute,&error))
                throw std::runtime_error(error);
            std::printf("netplay agreed tick=%u bytes=%zu machine=%u\n",match.checkpoint().next_tick,
                match.checkpoint().state.size(),launch.seat_machine[launch.local_seat]);
            std::printf("netplay replay=%llu view=%u\n",static_cast<unsigned long long>(match.replay_ticks()),
                static_cast<unsigned>(launch.view));
            std::printf("netplay saved pair=%s\n",checkpoint_absolute.string().c_str());
            end(closing,closing ? "" : "the other player left the match");
            const auto until=GbaNetplayPacer::Clock::now()+std::chrono::seconds(1);
            while (GbaNetplayPacer::Clock::now()<until) {
                if (launch.pump_lobby) launch.pump_lobby();
                window.pump(); match.poll(false); pacer.idle();
            }
            return 0;
        }
        const auto connection=match.connection();
        const auto status=connection.phase==GbaConnectionPhase::Reconnecting ?
            "Reconnecting "+gba_netplay_seat_names(connection.seats)+" ("+std::to_string((connection.grace_remaining_ms+999)/1000)+"s)" :
            match.phase()==GbaNetplayMatch::Phase::Starting ? std::string("Connecting") :
            (closing || (match.checkpoint_pending() && !boot.finish_tick)) ?
                std::string("Saving and leaving - close again to discard") :
            "Link cable - Player "+std::to_string(launch.seat_machine[launch.local_seat]+1)+" - Shift+F1 save and leave";
        const auto title=boot.title+" - "+status;
        if (title!=previous_title) { window.set_title(title.c_str()); previous_title=title; }
        if (match.take_output(output)) {
            // Canonical consoles keep no raster. After a mid-frame restore,
            // wait for a complete private presentation, while audio advances.
            if (const auto* pixels=presentation->pixels()) {
                const unsigned width=presentation->logical_width();
                if (!window.set_surface_size(width,160))
                    throw std::runtime_error("cannot resize netplay presentation surface");
                window.set_view_margins((width-240)/2,(width-240)-(width-240)/2,0,0);
                GbaNetplayPresentFrame frame;
                if (launch.present_frame) {
                    frame.guest_rgb=pixels;
                    frame.guest_width=presentation->width(); frame.guest_height=presentation->height();
                    frame.local_bus=&simulation.input_machine(launch.local_seat).bus;
                    frame.tick=match.next_tick(); frame.opening_end=script_end;
                    frame.leave_armed=GbaNetplayPacer::Clock::now()<leave_armed_until;
                    if (!window.drawable_size(&frame.drawable_width,&frame.drawable_height) ||
                        frame.drawable_width<=0 || frame.drawable_height<=0) {
                        frame.drawable_width=960; frame.drawable_height=540;   // headless test runs
                    }
                    if (launch.present_frame(frame) && frame.argb)
                        window.set_external_frame(frame.argb,frame.width,frame.height);
                    else frame.argb=nullptr;
                }
                window.present(pixels,presentation->width(),presentation->height());
                if (dump_prefix && ++presented_frames%60==0) {
                    const std::string name=std::string(dump_prefix)+"_"+std::to_string(presented_frames)+".ppm";
                    if (std::FILE* f=std::fopen(name.c_str(),"wb")) {
                        std::fprintf(f,"P6\n%u %u\n255\n",presentation->width(),presentation->height());
                        std::fwrite(pixels,3,static_cast<std::size_t>(presentation->width())*presentation->height(),f);
                        std::fclose(f);
                    }
                    // The game's own picture, when it drew one (prefix_<frame>_game.ppm).
                    if (frame.argb) {
                        const std::string game=std::string(dump_prefix)+"_"+std::to_string(presented_frames)+"_game.ppm";
                        if (std::FILE* f=std::fopen(game.c_str(),"wb")) {
                            std::fprintf(f,"P6\n%u %u\n255\n",frame.width,frame.height);
                            std::vector<std::uint8_t> rgb(static_cast<std::size_t>(frame.width)*frame.height*3);
                            for (std::size_t i=0;i<static_cast<std::size_t>(frame.width)*frame.height;++i) {
                                rgb[i*3]=std::uint8_t(frame.argb[i]>>16); rgb[i*3+1]=std::uint8_t(frame.argb[i]>>8);
                                rgb[i*3+2]=std::uint8_t(frame.argb[i]);
                            }
                            std::fwrite(rgb.data(),1,rgb.size(),f);
                            std::fclose(f);
                        }
                    }
                    // With each picture, every machine's EWRAM then IWRAM (prefix_<frame>_m<id>.ewram).
                    for (unsigned id=0;id<players;++id) {
                        const std::string mem=std::string(dump_prefix)+"_"+std::to_string(presented_frames)+
                            "_m"+std::to_string(id)+".ewram";
                        if (std::FILE* f=std::fopen(mem.c_str(),"wb")) {
                            std::fwrite(simulation.machine(id).bus.ewram_ptr(),1,0x40000,f);
                            std::fwrite(simulation.machine(id).bus.iwram_ptr(),1,0x8000,f);
                            // Then the IO registers 0x100..0x20F (timers,
                            // serial, IE/IF), read16 each: these reads have
                            // no side effects.
                            for (std::uint32_t off=0x100;off<0x210;off+=2) {
                                const std::uint16_t v=simulation.machine(id).bus.io().read16(off);
                                std::fwrite(&v,2,1,f);
                            }
                            std::fclose(f);
                        }
                    }
                }
            }
            std::size_t offset=0;
            for (const auto& block:output.audio_blocks) {
                window.push_audio_block(output.audio.data()+offset,block.count,block.rate);
                offset+=block.count;
            }
        }
        // Waiting for a peer: the consoles stand still and the title bar is
        // not seen in fullscreen, so the game shows the wait over the last
        // picture (a few times a second).
        if (connection.phase==GbaConnectionPhase::Reconnecting && launch.present_frame) {
            const auto t=GbaNetplayPacer::Clock::now();
            const auto* pixels=presentation->pixels();
            if (pixels && t>=next_wait_present) {
                next_wait_present=t+std::chrono::milliseconds(200);
                GbaNetplayPresentFrame frame;
                frame.guest_rgb=pixels;
                frame.guest_width=presentation->width(); frame.guest_height=presentation->height();
                frame.local_bus=&simulation.input_machine(launch.local_seat).bus;
                frame.tick=match.next_tick(); frame.opening_end=script_end;
                frame.leave_armed=t<leave_armed_until;
                frame.reconnect_seconds=(connection.grace_remaining_ms+999)/1000;
                if (!window.drawable_size(&frame.drawable_width,&frame.drawable_height) ||
                    frame.drawable_width<=0 || frame.drawable_height<=0) {
                    frame.drawable_width=960; frame.drawable_height=540;
                }
                if (launch.present_frame(frame) && frame.argb)
                    window.set_external_frame(frame.argb,frame.width,frame.height);
                window.present(pixels,presentation->width(),presentation->height());
            }
        }
        if (step==GbaNetplayMatch::Step::Idle) pacer.idle();
    }
    } catch (const std::exception& e) { end(false,e.what()); throw; }
}
}
std::shared_ptr<GbaNetplayLaunch> make_gba_netplay_launch(std::string program,std::string build,
    std::string content,void (*setup)(GbaInstance&)) {
    auto launch=std::make_shared<GbaNetplayLaunch>();
    launch->program_id=std::move(program); launch->build_identity=std::move(build);
    launch->content_sha256=std::move(content); launch->setup_instance=setup; launch->run=run;
    return launch;
}
}
