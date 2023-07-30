#ifndef TWICE_CONFIG_H
#define TWICE_CONFIG_H

#include <string>

namespace twice {

struct nds_config {
	std::string data_dir;
};

void set_logger_verbose_level(int level);

} // namespace twice

#endif
