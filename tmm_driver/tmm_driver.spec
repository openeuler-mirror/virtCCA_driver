# spec file for package tmm_driver
#
# Copyright (c) Huawei Technologies Co., Ltd. 2024. All rights reserved.
#
%global debug_package %{nil}
%define kmod_name tmm_driver
Name         :  %{kmod_name}-kmod
Summary      :  %{kmod_name}.ko
Version      :  1.0
Release      :  0.1
License      :  GPLV2
Group        :  System/Kernel
Source0      :  %{kmod_name}.tar.gz
BuildRoot    :  %{_tmppath}/%{kmod_name}-%{version}-build
BuildRequires:  dos2unix
Requires     :  rpm coreutils

%define module_dir 5.10.0-virtcca/tmm
%define module_ko       %{kmod_name}.ko

%description
%{name} module

%prep
%setup -q -n%{kmod_name}

%build
cd %_builddir/%{kmod_name}/src
make

%install
mkdir -p %{buildroot}/lib/modules/%{module_dir}
install -m 0640 %_builddir/%{kmod_name}/src/%{module_ko}        %{buildroot}/lib/modules/%{module_dir}

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root)
%attr(0640,root,root) /lib/modules/%{module_dir}/%{module_ko}

%post
insmod /lib/modules/%{module_dir}/%{module_ko}

%postun
rmmod %{module_ko}

%changelog
* Fri May 10 2024 huawei
    version 1.0 release 0.1