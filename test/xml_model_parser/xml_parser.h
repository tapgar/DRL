/*
 * xml_parser.h
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#ifndef XML_PARSER_H_
#define XML_PARSER_H_

#include <stdio.h>
#include "stdlib.h"
#include "stdbool.h"
#include <unistd.h>
#include "math.h"
#include <sstream>
#include <iostream>     // std::ios, std::istream, std::cout
#include <string>
#include <fstream>
#include <vector>
#include <iterator>

namespace XML_Parser {
	typedef struct site {
		unsigned int id;
		unsigned int body_id;
		std::string name;
		double pos[3];
		site() {id = body_id = 0;}
	} site;

	typedef struct joint {
		unsigned int id;
		unsigned int body_id;
		std::string name;
		std::string type;
		bool limited;
		double range[2];
		double ref;
		double damping;
		double armature;
		joint() {
			id = 0;
			body_id = 0;
			limited = true;
			damping = ref = armature = range[0] = range[1] = 0.0;
		}
	} joint;

	typedef struct {
		double pos[3];
		double inertia[6];
		double mass;
	} inertial;

	typedef struct body {
		unsigned int id;
		unsigned int p_id;
		std::string name;

		inertial I;
		double pos[3];
		double xyaxes[6];
		std::vector<joint> joints;
		body() {
			id = p_id = 0;
		}
	} body;


	void parse_string_number_array(std::vector<std::string> input, unsigned int start_idx, unsigned int end_idx, double* res)
	{
		if (end_idx - start_idx == 0)
		{
			res[0] = std::stod(input[start_idx].substr(input[start_idx].find_first_of("'")+1,input[start_idx].find_last_of("'")-input[start_idx].find_first_of("'")-1));
			return;
		}
		res[0] = std::stod(input[start_idx].substr(input[start_idx].find_first_of("'")+1));
		for (unsigned int i = start_idx+1; i < end_idx; i++)
			res[i-start_idx] = std::stod(input[i]);
		res[end_idx-start_idx] = std::stod(input[end_idx].substr(0,input[end_idx].find_last_of("'")));
	}

	int parse_xml_model(std::string xml_file, std::vector<body>* bodies, std::vector<site>* sites) {

		std::vector<unsigned int> p_ids;
		unsigned int b_id(0), j_id(0), s_id(0);
		p_ids.push_back(b_id);

		std::ifstream infile(xml_file);

		std::string line;
		while (std::getline(infile, line))
		{
			if (line.find("<!--") != std::string::npos)
				continue;
			else if (line.find("<body") != std::string::npos)
			{
				b_id++;
				body new_body;
				new_body.id = b_id;
				new_body.p_id = p_ids.back();

				std::istringstream iss(line);
				std::vector<std::string> results(std::istream_iterator<std::string>{iss},
												 std::istream_iterator<std::string>());
				new_body.name = results[1].substr(results[1].find_first_of("'")+1,results[1].find_last_of("'")-1-results[1].find_first_of("'"));

				parse_string_number_array(results, 2, 4, new_body.pos);
				if (results.size() > 5)
					parse_string_number_array(results, 5, 10, new_body.xyaxes);

				p_ids.push_back(b_id);
				bodies->push_back(new_body);
			}
			else if (line.find("</body") != std::string::npos)
			{
				p_ids.pop_back();
			}
			else if (line.find("<inertial") != std::string::npos)
			{
				std::istringstream iss(line);
				std::vector<std::string> results(std::istream_iterator<std::string>{iss},
												 std::istream_iterator<std::string>());
				parse_string_number_array(results, 1, 3, (bodies->back()).I.pos);
				parse_string_number_array(results, 4, 4, &((bodies->back()).I.mass));
				parse_string_number_array(results, 5, 10, (bodies->back()).I.inertia);
			}
			else if (line.find("<site") != std::string::npos)
			{
				std::istringstream iss(line);
				std::vector<std::string> results(std::istream_iterator<std::string>{iss},
												 std::istream_iterator<std::string>());
				site new_site;
				new_site.id = s_id;
				new_site.body_id = p_ids.back();
				s_id++;
				new_site.name = results[1].substr(results[1].find_first_of("'")+1,results[1].find_last_of("'")-1-results[1].find_first_of("'"));
				parse_string_number_array(results, 3, 5, new_site.pos);
				sites->push_back(new_site);
			}
			else if (line.find("<joint name") != std::string::npos)
			{
				joint new_joint;
				new_joint.id = j_id;
				new_joint.body_id = p_ids.back();
				new_joint.limited = true;
				new_joint.ref = 0.0;
				j_id++;
				std::istringstream iss(line);
				std::string token;
				while (std::getline(iss, token, '\''))
				{
					if (token.find("name=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						new_joint.name = token;
						continue;
					}
					if (token.find("type=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						new_joint.type = token;
						continue;
					}
					if (token.find("range=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						sscanf(token.c_str(), "%lf %lf", &(new_joint.range[0]), &(new_joint.range[1]));
						continue;
					}
					if (token.find("ref=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						new_joint.ref = std::stod(token);
						continue;
					}
					if (token.find("damping=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						new_joint.damping = std::stod(token);
						continue;
					}
					if (token.find("armature=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						new_joint.armature = std::stod(token);
						continue;
					}
					if (token.find("limited=") != std::string::npos)
					{
						std::getline(iss, token, '\'');
						if (token.find("false") != std::string::npos)
							new_joint.limited = false;
						continue;
					}
				}
				bodies->back().joints.push_back(new_joint);
			}
		}

		return 0;

	}

}

#endif /* XML_PARSER_H_ */
