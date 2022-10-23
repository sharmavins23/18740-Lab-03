#include "Cache.h"

#ifndef DEBUG_CACHE
#define debug(...)
#else
#define debug(...)                                   \
    do {                                             \
        printf("\033[36m[DEBUG] %s ", __FUNCTION__); \
        printf(__VA_ARGS__);                         \
        printf("\033[0m\n");                         \
    } while (0)
#endif

namespace ramulator {

    Cache::Cache(int size, int assoc, int block_size, int mshr_entry_num,
                 CacheLevel level, std::shared_ptr<CacheSystem> cachesys)
        : level(level),
          cachesys(cachesys),
          higher_cache(0),
          lower_cache(nullptr),
          size(size),
          assoc(assoc),
          block_size(block_size),
          mshr_entry_num(mshr_entry_num) {
        if (level == CacheLevel::L1) {
            level_string = "L1";
        } else if (level == CacheLevel::L2) {
            level_string = "L2";
        } else if (level == CacheLevel::L3) {
            level_string = "L3";
        }

        // Because we're running filtered L2 traces, *we can only have L3
        // cache*. Check the program output to confirm that level_string is
        // always L3 and you only get one such output (you shouldn't see L1 or
        // L2 as the level string ever). These parameters cannot be changed.
        // Your design must keep the same size, associativity, and block size!
        std::cout << "CacheLevel: " << level_string << ", Size: " << size
                  << ", Assoc: " << assoc << ", Block Size: " << block_size
                  << "\n";

        // Sanity check: whether everything is 2^N and size vs block_size makes
        // sense
        // You should not have to touch this
        assert((size & (size - 1)) == 0);
        assert((block_size & (block_size - 1)) == 0);
        assert((assoc & (assoc - 1)) == 0);
        assert(size >= block_size);

        // Initialize cache configuration
        set_num = size / (block_size * assoc);

        // set indexing mask
        index_mask = set_num - 1;

        // offset of an individual block
        index_offset = calc_log2(block_size);

        // offset of tag
        tag_offset = calc_log2(set_num) + index_offset;

        // #define DEBUG_CACHE on top of this file to switch debug output on
        // warning: this will generate a LOT of output. You may want to define
        // your own debug switch.
        debug("index_offset %d", index_offset);
        debug("index_mask 0x%x", index_mask);
        debug("tag_offset %d", tag_offset);

        // register stat counters.
        cache_read_miss.name(level_string + string("_cache_read_miss"))
            .desc("cache read miss count")
            .precision(0);

        cache_write_miss.name(level_string + string("_cache_write_miss"))
            .desc("cache write miss count")
            .precision(0);

        cache_total_miss.name(level_string + string("_cache_total_miss"))
            .desc("cache total miss count")
            .precision(0);

        cache_eviction.name(level_string + string("_cache_eviction"))
            .desc("number of evict from this level to lower level")
            .precision(0);

        cache_read_access.name(level_string + string("_cache_read_access"))
            .desc("cache read access count")
            .precision(0);

        cache_write_access.name(level_string + string("_cache_write_access"))
            .desc("cache write access count")
            .precision(0);

        cache_total_access.name(level_string + string("_cache_total_access"))
            .desc("cache total access count")
            .precision(0);

        cache_mshr_hit.name(level_string + string("_cache_mshr_hit"))
            .desc("cache mshr hit count")
            .precision(0);
        cache_mshr_unavailable
            .name(level_string + string("_cache_mshr_unavailable"))
            .desc("cache mshr not available count")
            .precision(0);
        cache_set_unavailable
            .name(level_string + string("_cache_set_unavailable"))
            .desc("cache set not available")
            .precision(0);
    }

    // dispatch requests to the cache system
    bool Cache::send(Request req) {
        debug("level %d req.addr %lx req.type %d, index %d, tag %ld",
              int(level), req.addr, int(req.type), get_index(req.addr),
              get_tag(req.addr));

        // handle some stats
        cache_total_access++;
        if (req.type == Request::Type::WRITE) {
            cache_write_access++;
        } else {
            assert(req.type == Request::Type::READ);
            cache_read_access++;
        }

        // Find the lines corresponding to the request addr.
        // If those lines don't exist, get_lines() will allocate them! (check
        // refresh_lru_lower() below for how to just lookup). This will not
        // overrun because there are only so many possible line addrs. Note:
        // this reference can modify the original data structure.
        auto& lines = get_lines(req.addr);

        // iterators are generalized pointers
        std::list<Line>::iterator line;

        // tag matched?
        if (is_hit(lines, req.addr, &line)) {
            // Line list is a LRU queue. Update the queue and dirty bit by
            // pushing a new line in
            lines.push_back(
                Line(req.addr, get_tag(req.addr), false,
                     line->dirty || (req.type == Request::Type::WRITE)));
            // remove old line...basically replaced the old line with new line
            lines.erase(line);
            // let the system know that a hit happened at current level with
            // (always L3 for us)'s latency
            cachesys->hit_list.push_back(
                make_pair(cachesys->clk + latency[int(level)], req));

            debug("hit, update timestamp %ld", cachesys->clk);
            debug("hit finish time %ld", cachesys->clk + latency[int(level)]);

            return true;

        } else {
            // miss, record stats and try MSHR
            debug("miss @level %d", int(level));
            cache_total_miss++;
            if (req.type == Request::Type::WRITE) {
                cache_write_miss++;
            } else {
                assert(req.type == Request::Type::READ);
                cache_read_miss++;
            }

            // The dirty bit will be set if this is a write request and @L1
            bool dirty = (req.type == Request::Type::WRITE);

            // Modify the type of the request to lower level
            // (write miss will cause a read to lower levels)
            if (req.type == Request::Type::WRITE) {
                req.type = Request::Type::READ;
            }

            assert(req.type == Request::Type::READ);

            // Look it up in MSHR entries
            // if the req was already in MSHR, update stat and quit out (already
            // recorded miss)
            auto mshr = hit_mshr(req.addr);
            if (mshr != mshr_entries.end()) {
                debug("hit mshr");
                cache_mshr_hit++;
                mshr->second->dirty = dirty || mshr->second->dirty;
                return true;
            }

            // All requests come to this stage will be READ and not in MSHR, so
            // they should be recorded in MSHR entries.
            if (mshr_entries.size() == mshr_entry_num) {
                // When no MSHR entries available, the miss request
                // is stalling.
                cache_mshr_unavailable++;
                debug("no mshr entry available");
                return false;
            }

            // MSHR entry needs a line
            // Check whether everything is locked
            if (all_sets_locked(lines)) {
                cache_set_unavailable++;
                return false;
            }

            // try to allocate a line
            auto newline = allocate_line(lines, req.addr);
            if (newline == lines.end()) {
                return false;
            }

            newline->dirty = dirty;

            // Add to MSHR entries
            mshr_entries.push_back(make_pair(req.addr, newline));

            // Send the request to next level
            if (!is_last_level) {
                if (!lower_cache->send(req)) {
                    retry_list.push_back(req);
                }
            } else {
                // for L3, push req into waiting list to send to memory
                cachesys->wait_list.push_back(
                    make_pair(cachesys->clk + latency[int(level)], req));
            }
            return true;
        }
    }

    // sets the given addr to the back of the LRU queue, updates dirty bit
    void Cache::refresh_lru_lower(long addr, bool dirty) {
        // get the lines for a certain addr
        auto line_list = cache_lines.find(get_index(addr));

        // line requested for eviction must exist
        assert(line_list != cache_lines.end());

        // search for a tag match. find_if() finds an entry if the lambda
        // function you provide is true for that entry. capture "this" to use
        // get_tag() when searching
        auto& lines = line_list->second;
        auto line = find_if(lines.begin(), lines.end(), [addr, this](Line l) {
            return (l.tag == get_tag(addr));
        });

        // line requested for eviction must exist
        assert(line != lines.end());

        // Line list is a LRU queue. Update the queue by pushing a new line in
        lines.push_back(Line(addr, get_tag(addr), false, dirty || line->dirty));

        // remove the old line object
        lines.erase(line);
    }

    std::pair<long, bool> Cache::invalidate(long addr) {
        // delay of current cache level
        long delay = latency_each[int(level)];
        bool dirty = false;

        // lookup line
        auto& lines = get_lines(addr);
        if (lines.size() == 0) {
            // The line of this address doesn't exist.
            return make_pair(0, false);
        }

        auto line = find_if(lines.begin(), lines.end(), [addr, this](Line l) {
            return (l.tag == get_tag(addr));
        });

        // If the line is in this level cache, then erase it from
        // the buffer.
        if (line != lines.end()) {
            assert(!line->lock);
            debug("invalidate %lx @ level %d", addr, int(level));
            lines.erase(line);
        } else {
            // If it's not in current level, then no need to go up.
            return make_pair(delay, false);
        }

        // run invaldation on higher levels
        if (higher_cache.size()) {
            long max_delay = delay;
            for (auto hc : higher_cache) {
                auto result = hc->invalidate(addr);
                if (result.second) {
                    max_delay = max(max_delay, delay + result.first * 2);
                } else {
                    max_delay = max(max_delay, delay + result.first);
                }
                dirty = dirty || line->dirty || result.second;
            }
            delay = max_delay;
        } else {
            dirty = line->dirty;
        }
        return make_pair(delay, dirty);
    }

    void Cache::evict(std::list<Line>* lines,
                      std::list<Line>::iterator victim) {
        debug("level %d miss evict victim %lx", int(level), victim->addr);
        cache_eviction++;

        long addr = victim->addr;
        long invalidate_time = 0;
        bool dirty = victim->dirty;

        // First invalidate the victim line in higher level.
        if (higher_cache.size()) {
            for (auto hc : higher_cache) {
                auto result = hc->invalidate(addr);
                invalidate_time =
                    max(invalidate_time,
                        result.first +
                            (result.second ? latency_each[int(level)] : 0));
                dirty = dirty || result.second || victim->dirty;
            }
        }

        debug("invalidate delay: %ld, dirty: %s", invalidate_time,
              dirty ? "true" : "false");

        if (!is_last_level) {
            // not LLC eviction
            assert(lower_cache != nullptr);
            // don't evict from lower level, just refresh the LRU queue and
            // dirty bit for that line
            lower_cache->refresh_lru_lower(addr, dirty);
        } else {
            // LLC eviction...create write request to memory in wait list
            if (dirty) {
                Request write_req(addr, Request::Type::WRITE);
                cachesys->wait_list.push_back(make_pair(
                    cachesys->clk + invalidate_time + latency[int(level)],
                    write_req));

                debug(
                    "inject one write request to memory system "
                    "addr %lx, invalidate time %ld, issue time %ld",
                    write_req.addr, invalidate_time,
                    cachesys->clk + invalidate_time + latency[int(level)]);
            }
        }

        lines->erase(victim);
    }

    std::list<Line>::iterator Cache::allocate_line(std::list<Line>& lines,
                                                   long addr) {
        // See if an eviction is needed
        if (need_eviction(lines, addr)) {
            // Victim line should be unlocked at current and higher levels
            // capture "this" to use check_unlock()
            auto victim =
                find_if(lines.begin(), lines.end(), [this](Line line) {
                    bool check = !line.lock;
                    if (!is_first_level) {
                        for (auto hc : higher_cache) {
                            check = check && hc->check_unlock(line.addr);
                        }
                    }
                    return check;
                });

            // couldn't find a suitable line to evict
            if (victim == lines.end()) {
                return lines.end();
            }

            assert(victim != lines.end());

            evict(&lines, victim);
        }

        // Allocate new line with lock bit on and dirty bit off
        lines.push_back(Line(addr, get_tag(addr)));
        auto last_element = lines.end();

        // points to the entry just pushed back i.e. the newly allocated line
        --last_element;
        return last_element;
    }

    bool Cache::is_hit(std::list<Line>& lines, long addr,
                       std::list<Line>::iterator* pos_ptr) {
        auto pos = find_if(lines.begin(), lines.end(), [addr, this](Line l) {
            return (l.tag == get_tag(addr));
        });
        *pos_ptr = pos;
        if (pos == lines.end()) {
            return false;
        }
        return !pos->lock;
    }

    void Cache::concatlower(Cache* lower) {
        lower_cache = lower;
        assert(lower != nullptr);
        lower->higher_cache.push_back(this);
    };

    bool Cache::need_eviction(const std::list<Line>& lines, long addr) {
        if (find_if(lines.begin(), lines.end(), [addr, this](Line l) {
                return (get_tag(addr) == l.tag);
            }) != lines.end()) {
            // Due to MSHR, the program can't reach here. Just for checking
            assert(false);
        } else {
            if (lines.size() < assoc) {
                return false;
            } else {
                return true;
            }
        }
    }

    void Cache::callback(Request& req) {
        debug("level %d", int(level));

        auto it =
            find_if(mshr_entries.begin(), mshr_entries.end(),
                    [&req, this](
                        std::pair<long, std::list<Line>::iterator> mshr_entry) {
                        return (align(mshr_entry.first) == align(req.addr));
                    });

        if (it != mshr_entries.end()) {
            it->second->lock = false;
            mshr_entries.erase(it);
        }

        if (higher_cache.size()) {
            for (auto hc : higher_cache) {
                hc->callback(req);
            }
        }
    }

    void Cache::tick() {
        if (!lower_cache->is_last_level) lower_cache->tick();

        for (auto it = retry_list.begin(); it != retry_list.end(); it++) {
            if (lower_cache->send(*it)) it = retry_list.erase(it);
        }
    }

    void CacheSystem::tick() {
        debug("clk %ld", clk);

        ++clk;

        // Sends ready waiting request to memory
        auto it = wait_list.begin();
        while (it != wait_list.end() && clk >= it->first) {
            if (!send_memory(it->second)) {
                ++it;
            } else {
                debug("complete req: addr %lx", (it->second).addr);

                it = wait_list.erase(it);
            }
        }

        // hit request callback
        it = hit_list.begin();
        while (it != hit_list.end()) {
            if (clk >= it->first) {
                it->second.callback(it->second);

                debug("finish hit: addr %lx", (it->second).addr);

                it = hit_list.erase(it);
            } else {
                ++it;
            }
        }
    }

}  // namespace ramulator