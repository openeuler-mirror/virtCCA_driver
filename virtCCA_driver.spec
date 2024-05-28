# spec file for package virtCCA_driver
#
# Copyright (c) Huawei Technologies Co., Ltd. 2024. All rights reserved.
#
%global debug_package %{nil}
%define kmod_name virtCCA_driver
Name         :  %{kmod_name}-kmod
Summary      :  All drivers for virtCCA
Version      :  1.0
Release      :  0.1
License      :  GPLV2
Group        :  System/Kernel
Source0      :  %{kmod_name}.tar.gz
BuildRoot    :  %{_tmppath}/%{kmod_name}-%{version}-build
BuildRequires:  dos2unix
Requires     :  rpm coreutils

%define module_dir 5.10.0-virtcca
%define tmm_ko       tmm_driver.ko
%define huk_ko       huk_derive.ko

%description
%{name} module

%prep
%setup -q -n%{kmod_name}

%build
cd %_builddir/%{kmod_name}
make

%install
mkdir -p %{buildroot}/lib/modules/%{module_dir}
install -m 0640 %_builddir/%{kmod_name}/tmm_driver/src/%{tmm_ko}        %{buildroot}/lib/modules/%{module_dir}
install -m 0640 %_builddir/%{kmod_name}/huk_derive/src/%{huk_ko}        %{buildroot}/lib/modules/%{module_dir}

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root)
%attr(0640,root,root) /lib/modules/%{module_dir}/%{tmm_ko}
%attr(0640,root,root) /lib/modules/%{module_dir}/%{huk_ko}

%post
insmod /lib/modules/%{module_dir}/%{tmm_ko}
insmod /lib/modules/%{module_dir}/%{huk_ko}

%postun
rmmod %{module_ko}

%changelog
* Fri May 10 2024 huawei
    version 1.0 release 0.1