// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file TypeCodeVariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_BASE_COLA2_TYPECODEVARIABLECOMMAND_H
#define SICK_SAFETYSCANNERS_BASE_COLA2_TYPECODEVARIABLECOMMAND_H


#include <sick_safetyscanners_base/cola2/VariableCommand.h>
#include <sick_safetyscanners_base/data_processing/ParseTypeCodeData.h>
#include <sick_safetyscanners_base/datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the type code of the sensor.
 */
class TypeCodeVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;

  /*!
   * \brief Constructor of the Command.
   *
   * \param session The current cola2 session.
   * \param type_code Reference to the type code variable, which will be written on execution.
   */
  TypeCodeVariableCommand(Cola2Session& session, datastructure::TypeCode& type_code);

  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ParseTypeCodeData> m_type_code_parser_ptr;

  sick::datastructure::TypeCode& m_type_code;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_BASE_COLA2_TYPECODEVARIABLECOMMAND_H
