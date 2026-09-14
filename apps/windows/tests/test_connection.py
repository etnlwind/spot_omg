import asyncio
from types import SimpleNamespace
import pytest
from spot_controller.connection import BleLink, TcpLink


def test_tcp_identity_and_eof():
    async def scenario():
        async def server(reader,writer):
            assert await reader.readline()==b'identity\n'
            writer.write(b'$SPOTBACKEND backend=sim protocol=1\r\n# ')
            await writer.drain()
            assert await reader.readline()==b'syncstate\n'
            writer.write(b'$SPOTSTATE pose=stand\r\n# ')
            await writer.drain()
            writer.close();await writer.wait_closed()
        listener=await asyncio.start_server(server,'127.0.0.1',0)
        link=TcpLink('127.0.0.1',listener.sockets[0].getsockname()[1])
        try:
            await link.open();await link.write(b'syncstate\n')
            assert b'$SPOTSTATE' in await link.read()
            with pytest.raises(ConnectionError):await link.read()
        finally:
            await link.close();listener.close();await listener.wait_closed()
    asyncio.run(scenario())


def test_tcp_rejects_hardware_identity_before_any_motion():
    async def scenario():
        received=[]
        async def server(reader,writer):
            received.append(await reader.readline())
            writer.write(b'$SPOTBACKEND backend=robot protocol=1\r\n# ');await writer.drain()
            writer.close();await writer.wait_closed()
        listener=await asyncio.start_server(server,'127.0.0.1',0)
        link=TcpLink('127.0.0.1',listener.sockets[0].getsockname()[1])
        try:
            with pytest.raises(ConnectionError):await link.open()
            assert received==[b'identity\n']
        finally:
            await link.close();listener.close();await listener.wait_closed()
    asyncio.run(scenario())


def test_ble_chunks_use_negotiated_limit_and_await_each_write():
    async def scenario():
        link=BleLink(); sent=[]
        class Client:
            async def write_gatt_char(self,char,data,response):
                sent.append((data,response))
        link.client=Client();link.response=False
        link.characteristic=SimpleNamespace(max_write_without_response_size=20)
        data=b'log time 12345678901234567890\n'
        await link.write(data)
        assert all(len(d)<=20 and not r for d,r in sent)
        assert b''.join(d for d,r in sent)==data
        assert link.service=='6e400001-b5a3-f393-e0a9-e50e24dcca9e'
        assert BleLink(True).service=='6e400101-b5a3-f393-e0a9-e50e24dcca9e'
    asyncio.run(scenario())
