/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2011 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _CB_LAB_HANDLER_
#define _CB_LAB_HANDLER_

#include <qxml.h>
#include <webots/Supervisor.hpp>

#include "cblab.h"

class QString;

/**
 * SAX parser of labirinth file
 */
class cbLabHandler : public QXmlDefaultHandler
{
public:
    bool startDocument();
    bool endDocument();
    bool startElement( const QString&, const QString&, const QString& , const QXmlAttributes& );
    bool endElement( const QString&, const QString&, const QString& );
	void setDocumentLocator(QXmlLocator *);
    void setChildrenField(webots::Field *field);

    cbLab *getLab() { return lab; }


private:    
    webots::Field *children_field;
    cbLab *lab;
    cbWall *wall;
    int target_id=0;
};     

#endif
