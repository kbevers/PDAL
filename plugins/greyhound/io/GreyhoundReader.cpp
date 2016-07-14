/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "GreyhoundReader.hpp"
#include "bbox.hpp"
#include "dir.hpp"
#include <pdal/pdal_macros.hpp>
#include <pdal/Compression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html" );

CREATE_SHARED_PLUGIN(1, 0, GreyhoundReader, Reader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

GreyhoundReader::GreyhoundReader()
    : Reader()
    , m_url()
    , m_resource()
    , m_numPoints(0)
    , m_index(0)
    , m_depthBegin(0)
    , m_depthEnd(std::numeric_limits<uint32_t>::max())
    , m_baseDepth(0)
{ }

GreyhoundReader::~GreyhoundReader()
{
}

DimTypeList GreyhoundReader::getSchema(const Json::Value& jsondata) const
{
    DimTypeList output;

    if (jsondata.isMember("schema") &&
        jsondata["schema"].isArray())
    {
        Json::Value jsonDimArray(jsondata["schema"]);

        for (std::size_t i(0); i < jsonDimArray.size(); ++i)
        {
            const Json::Value& jsonDim(
                    jsonDimArray[static_cast<Json::ArrayIndex>(i)]);

            const Dimension::Id id(
                    Dimension::id(jsonDim["name"].asString()));

            const Dimension::Type type(
                static_cast<Dimension::Type>(
                    static_cast<int>(Dimension::fromName(
                        jsonDim["type"].asString())) |
                    std::stoi(jsonDim["size"].asString())));

            DimType d;
            d.m_id = id;
            d.m_type = type;
            output.push_back(d);
        }
    }

    return output;
}
BOX3D GreyhoundReader::getBounds(const Json::Value& jsondata) const
{
    BOX3D output;
    if (jsondata.isMember("boundsConforming") &&
        jsondata["boundsConforming"].isArray())
    {
        Json::Value bounds(jsondata["boundsConforming"]);

        output.minx = bounds[0].asDouble();
        output.miny = bounds[1].asDouble();
        output.minz = bounds[2].asDouble();
        output.maxx = bounds[3].asDouble();
        output.maxy = bounds[4].asDouble();
        output.maxz = bounds[5].asDouble();
    }
    else
    {
        throw pdal_error("Greyhound info response has no \"bounds\" member");
    }

    return output;
}

Json::Value GreyhoundReader::getResourceInfo()
{
    std::string info_url = m_url + "/resource/" + m_resource + "/info";
//     log()->get(LogLevel::Info) << "fetching info URL " << info_url;

    arbiter::Arbiter a;
    auto response = a.get(info_url);

    Json::Value jsonResponse;
    Json::Reader jsonReader;
    jsonReader.parse(response, jsonResponse);

    return jsonResponse;

}


void GreyhoundReader::initialize(PointTableRef table)
{
    m_resourceInfo = getResourceInfo();

    m_dimData = getSchema(m_resourceInfo);
    m_bounds = getBounds(m_resourceInfo);

    std::string srs = m_resourceInfo["srs"].asString();
    setSpatialReference(SpatialReference(srs));

    m_baseDepth = m_resourceInfo["baseDepth"].asUInt();

    PointLayoutPtr layout = table.layout();

}

QuickInfo GreyhoundReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    PointTable table;
    initialize(table);
    addDimensions(layout.get());

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    qi.m_srs = getSpatialReference();
    qi.m_valid = true;
    qi.m_pointCount = estimatePointCount();
    qi.m_bounds = m_bounds;

    done(table);

    return qi;
}

void GreyhoundReader::addArgs(ProgramArgs& args)
{
    args.add("url", "URL", m_url);
    args.add("resource", "Resource ID", m_resource);
    args.add("bounds", "Bounding cube", m_bounds);
    args.add("depth_begin", "Beginning depth to query", m_depthBegin);
    args.add("depth_end", "Ending depth to query", m_depthEnd);
}


void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& dim: m_dimData)
    {
        layout->registerDim(dim.m_id, dim.m_type);
    }
}

uint64_t sumHierarchy(Json::Value tree)
{

    uint64_t output(0);
    if (!tree.isMember("n")) return output;

    output += tree["n"].asUInt64();

    auto summarize = [tree](const std::string& name)
    {
        uint64_t output(0);
        if (tree.isMember(name))
        {
            output += tree[name]["n"].asUInt64();
            output += sumHierarchy(tree[name]);
        }
        return output;
    };

    output += summarize("nwu");
    output += summarize("neu");
    output += summarize("swu");
    output += summarize("seu");

    return output;
}

point_count_t GreyhoundReader::estimatePointCount()  const
{
    // Estimate the number of points
    pdal::greyhound::Point minimum;
    pdal::greyhound::Point maximum;

    minimum.x = m_bounds.minx;
    minimum.y = m_bounds.miny;
    minimum.z = m_bounds.minz;

    maximum.x = m_bounds.maxx;
    maximum.y = m_bounds.maxy;
    maximum.z = m_bounds.maxz;

    pdal::greyhound::BBox queryBounds(minimum, maximum, true);
    pdal::greyhound::BBox currentBounds(m_resourceInfo["bounds"]);

    int split(0);
    while (currentBounds.contains(queryBounds))
    {
        currentBounds.go(pdal::greyhound::getDirection(currentBounds.mid(),queryBounds.mid()));
        split++;
    }

    log()->get(LogLevel::Info) << "split count: " << split << std::endl;

    std::stringstream bounds;
    bounds << "[" << currentBounds.min().x << "," << currentBounds.min().y << "," << currentBounds.min().z;
    bounds << "," << currentBounds.max().x << "," << currentBounds.max().y << "," << currentBounds.max().z << "]";

    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/hierarchy?bounds=" << arbiter::http::sanitize(bounds.str());
    url << "&depthBegin=" << m_baseDepth + split;
    url << "&depthEnd=" << m_baseDepth + split + 6;

    log()->get(LogLevel::Info) << "fetching hierarchy URL " << url.str() << std::endl;

    arbiter::Arbiter a;
    auto response = a.get(url.str());

    Json::Value jsonResponse;
    Json::Reader jsonReader;
    jsonReader.parse(response, jsonResponse);
    uint64_t count = sumHierarchy(jsonResponse);
    log()->get(LogLevel::Info) << "estimated count " << count << std::endl;;
    return count;
}

void GreyhoundReader::ready(PointTableRef)
{


}

point_count_t GreyhoundReader::read(
        PointViewPtr view,
        const point_count_t count)
{

    std::stringstream bounds;
    bounds << "[" << m_bounds.minx << "," << m_bounds.minx << "," << m_bounds.minz;
    bounds << "," << m_bounds.maxx << "," << m_bounds.maxy << "," << m_bounds.maxz << "]";

    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/read?bounds=" << arbiter::http::sanitize(bounds.str());
    url << "&depthBegin=" << std::max(m_depthBegin, m_baseDepth);
    url << "&depthEnd=" << m_depthEnd;

#ifdef PDAL_HAVE_LAZPERF
    url << "&compress=true";
#endif

    log()->get(LogLevel::Info) << "fetching read URL " << url.str() << std::endl;

    arbiter::Arbiter a;
    auto response = a.getBinary(url.str());

    PointId nextId = view->size();
    point_count_t numRead = 0;

    log()->get(LogLevel::Info) << "Fetched "
                               << response.size()
                               << " bytes from "
                               << m_url << std::endl;

    const uint32_t numPoints = *reinterpret_cast<const uint32_t*>(response.data() + response.size() - sizeof(uint32_t));

    log()->get(LogLevel::Info) << "Fetched "
                               << numPoints
                               << " points from "
                               << m_url << std::endl;

#ifdef PDAL_HAVE_LAZPERF
    SignedLazPerfBuf buf(response);
    LazPerfDecompressor<SignedLazPerfBuf> decompressor(buf, m_dimData);

    std::vector<char> ptBuf(decompressor.pointSize());
    while (numRead < numPoints)
    {
        char* outbuf = ptBuf.data();
        point_count_t numWritten =
            decompressor.decompress(outbuf, ptBuf.size());

        for (auto di = m_dimData.begin(); di != m_dimData.end(); ++di)
        {
            view->setField(di->m_id, di->m_type, nextId, outbuf);
            outbuf += Dimension::size(di->m_type);
        }
        if (m_cb)
            m_cb(*view, nextId);
        nextId++;
        numRead++;
    }
#else

    raise pdal_error("uncompressed not implemented!");
#endif
    return numRead;
}

bool GreyhoundReader::eof() const
{
    return m_index >= m_numPoints;
}

void GreyhoundReader::done(PointTableRef)
{
}

} // namespace pdal

