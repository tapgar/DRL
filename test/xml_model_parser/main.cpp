/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "xml_parser.h"


int main() {

	std::vector<XML_Parser::body> bodies;
	std::vector<XML_Parser::site> sites;

	XML_Parser::parse_xml_model("/home/tapgar/cuda-workspace/DRL/models/singleleg.xml", &bodies, &sites);

	return 0;

}



